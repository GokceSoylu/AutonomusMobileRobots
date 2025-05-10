from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt

client = RemoteAPIClient()
sim = client.require('sim')

# === Object handles ===
sensors = {
    "left": sim.getObject('/LeftSensor'),
    "right": sim.getObject('/RightSensor'),
    "mid": sim.getObject('/MiddleSensor')
}
motors = {
    "left": sim.getObject('/DynamicLeftJoint'),
    "right": sim.getObject('/DynamicRightJoint')
}
vision_sensor = sim.getObject('/Vision_sensor')
proximity_sensor = sim.getObject('/Proximity_sensor')
linetracer = sim.getObject('/LineTracer')

# === Parameters ===
dmax = 0.35
Kp, Kd, Ki =  0.18, 0.015 ,  0.004
e_old = 0
e_integral = 0
v_base = 0.5
d_obstacle= 0.30

def get_image(sensor):
    img, res = sim.getVisionSensorImg(sensor)
    if img:
        img = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return None

def sensor_ir(sensor):
    img, _ = sim.getVisionSensorImg(sensor)
    return np.frombuffer(img, dtype=np.uint8)[0] > 40 if img else False

def get_distance(sensor):
    state, point, *_ = sim.readProximitySensor(sensor)
    return np.linalg.norm(point) if state else dmax

def execute():
    global e_old, e_integral
    state = "FOLLOWING"
    vL_list, vR_list, px, py = [], [], [], []

    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1)

    while sim.getSimulationState() != sim.simulation_stopped:

        pos = sim.getObjectPosition(linetracer, -1)
        px.append(pos[0])
        py.append(pos[1])

        d = get_distance(proximity_sensor)
        imgL, imgM, imgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])

        if state == "FOLLOWING":

            if d < d_obstacle:
                print("FOLLOWING: Obstacle detected")
                state = "AVOIDANCE"
                continue
            
            
            if imgL and not imgM and not imgR:
                e_new = -1
            elif imgL and imgM and not imgR:
                e_new = -0.5
            elif imgR and not imgM and not imgL:
                e_new = 1
            elif imgR and imgM and not imgL:
                e_new = 0.5
            elif imgM and not imgL and not imgR:
                e_new = 0
            else:
                e_new = 0

            adaptive_Kp = Kp * 1.5 if abs(e_new) > 0 else Kp
            adaptive_Kd = Kd * 1.5 if abs(e_new) > 0 else Kd

            e_integral += e_new
            correction = adaptive_Kp * e_new + adaptive_Kd * (e_new - e_old) + Ki * e_integral
            e_old = e_new

            vL = v_base - correction
            vR = v_base + correction

            sim.setJointTargetVelocity(motors["left"], vL)
            sim.setJointTargetVelocity(motors["right"], vR)

            vL_list.append(vL)
            vR_list.append(vR)
            print("FOLLOWING:", vL, vR)

        elif state == "AVOIDANCE":
            t_start = sim.getSimulationTime()
            while sim.getSimulationTime() - t_start < 0.8:
                print("AVOIDANCE: turning ")
                sim.setJointTargetVelocity(motors["left"], -0.5)
                sim.setJointTargetVelocity(motors["right"], 0.5)
                imgL, imgM, imgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])
                found_line = not(imgL or imgR or imgM)
                if found_line:
                    print("AVOIDANCE: Line detected")
                    state = "SEARCH"
                    sim.setJointTargetVelocity(motors["left"], 0.7)
                    sim.setJointTargetVelocity(motors["right"], -0.7)
                    break
                
            t_start = sim.getSimulationTime()
            while sim.getSimulationTime() - t_start < 0.8:
                print("AVOIDANCE: moving forward.")
                sim.setJointTargetVelocity(motors["left"], 0.5)
                sim.setJointTargetVelocity(motors["right"], 0.5)
                imgL, imgM, imgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])
                found_line = not(imgL or imgR or imgM)
                if found_line:
                    print("AVOIDANCE: Line detected")
                    state = "SEARCH"
                    sim.setJointTargetVelocity(motors["left"], 0.7)
                    sim.setJointTargetVelocity(motors["right"], -0.7)
                    break
            state = "SEARCH"
            

        elif state == "SEARCH":
            search_duration = 5
            search_start_time = sim.getSimulationTime()
            while sim.getSimulationTime() - search_start_time < search_duration:
                d = get_distance(proximity_sensor)
                imgL, imgM, imgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])
                found_line = not((imgL or imgR) and imgM)
                print("SEARCH:",d,found_line, sim.getSimulationTime() - search_start_time)
                sim.setJointTargetVelocity(motors["left"], 0.2)
                sim.setJointTargetVelocity(motors["right"], -0.2)
                
                if d < d_obstacle:
                    print("SEARCH: Obstacle detected")
                    state = "AVOIDANCE"
                    break
                elif found_line:
                    print("SEARCH: Line found")
                    state = "ALIGNMENT"
                    break
                else:
                    print("SEARCH: else")
                
        elif state == "ALIGNMENT":
            fimgL, fimgM, fimgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])
            state=0
            if not(fimgR and fimgM and fimgL):
                state=1
            elif not(fimgR and fimgM):
                state=2
            elif not(fimgL and fimgM):
                state=3
            while True:
                sim.setJointTargetVelocity(motors["left"], 0.0 )
                sim.setJointTargetVelocity(motors["right"], 0.2)
                imgL, imgM, imgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])
                print("ALIGNMENT:",imgL , imgM ,imgR, state)
                
                if not(imgR and imgM) and (state==0):
                    break
                elif ( (imgL and imgM) and not(imgR) )  and (state==1):
                    break
                elif not(imgL and imgM) and (state==2):
                    break
                
            state = "FOLLOWING"

        img = get_image(vision_sensor)
        if img is not None:
            cv2.imshow('Front Cam', img)
            if cv2.waitKey(5) & 0xFF == 27:
                break

        ax1.clear()
        ax1.scatter(py, px)
        ax1.set_xlabel("X axis")
        ax1.set_ylabel("Y axis")
        ax1.set_title("Robot Path")
        ax1.grid()

        ax2.clear()
        ax2.plot(vL_list, label="Left Motor")
        ax2.plot(vR_list, label="Right Motor")
        ax2.set_title("Motor Speeds")
        ax2.legend()
        ax2.grid()

        plt.pause(0.0001)

    cv2.destroyAllWindows()

def start_simulation():
    sim.startSimulation()
    print("Starting simulation... Waiting to start...")
    while sim.getSimulationState() == sim.simulation_stopped:
        print("Waiting: sim.getSimulationState() == sim.simulation_stopped")
        time.sleep(0.1)
    execute()
    sim.stopSimulation()

if __name__ == '__main__':
    start_simulation()
