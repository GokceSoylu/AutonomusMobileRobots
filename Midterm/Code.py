
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
Kp, Kd, Ki =  0.22, 0.02 ,  0.003
e_old = 0
e_integral = 0
v_base = 0.6
d_obstacle=0.2
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
    state = "LINE_FOLLOWING"
    vL_list, vR_list, px, py = [], [], [], []

    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1)

    
    start_time = time.time()
    lap1_recorded = False
    lap2_recorded = False

    def distance_from_start(pos):
        return (pos[0]**2 + pos[1]**2)**0.5

    while sim.getSimulationState() != sim.simulation_stopped:
        pos = sim.getObjectPosition(linetracer, -1)
        px.append(pos[0])
        py.append(pos[1])

        current_dist = distance_from_start(pos)
        now = time.time()

        if not lap1_recorded and current_dist < 0.1 and now - start_time > 3:
            print(f"1st Lap Time: {now - start_time:.2f} sec")
            lap1_time = now - start_time
            lap1_recorded = True

        if lap1_recorded and not lap2_recorded and current_dist < 0.1 and now - start_time > 8:
            print(f"2nd Lap Time: {now - start_time:.2f} sec")
            lap2_time = now - start_time
            lap2_recorded = True

        pos = sim.getObjectPosition(linetracer, -1)
        px.append(pos[0])
        py.append(pos[1])

        d = get_distance(proximity_sensor)
        imgL, imgM, imgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])
        
        if state == "LINE_FOLLOWING":
            if d < d_obstacle:
                print("Obstacle detected")
                state = "OBSTACLE_AVOIDANCE"
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
                e_new = 0  # fallback


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

        elif state == "OBSTACLE_AVOIDANCE":
            print("Avoiding obstacle: turning + moving forward.")
            sim.setJointTargetVelocity(motors["left"], 0.5)
            sim.setJointTargetVelocity(motors["right"], -0.5)
            time.sleep(0.5)

            sim.setJointTargetVelocity(motors["left"], 0.5)
            sim.setJointTargetVelocity(motors["right"], 0.5)
            time.sleep(0.6)

            state = "LINE_SEARCH"
            search_start_time = time.time()

        elif state == "LINE_SEARCH":
            print("Line search mode. Rotating")
            search_duration = 5.0

            while time.time() - search_start_time < search_duration:
                d = get_distance(proximity_sensor)
                imgL, imgM, imgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])
                found_line = not((imgL or imgR) and imgM)

                sim.setJointTargetVelocity(motors["left"], 0.25)
                sim.setJointTargetVelocity(motors["right"], -0.25)

                if d < d_obstacle:
                    print("Obstacle detected during search")
                    state = "OBSTACLE_AVOIDANCE"
                    break
                elif found_line:
                    print("Line found")
                    state = "LINE_ALIGNMENT"
                    break
                time.sleep(0.01)
            else:
                print("Line search failed. Trying alternative avoidance.")
                sim.setJointTargetVelocity(motors["left"], 0.5)
                sim.setJointTargetVelocity(motors["right"], -0.5)
                time.sleep(0.5)
                state = "LINE_SEARCH"
                search_start_time = time.time()

        elif state == "LINE_ALIGNMENT":
            imgL, imgM, imgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])
            found_line = 1
            while found_line:
                if imgL and not imgM and not imgR:
                    sim.setJointTargetVelocity(motors["left"], 0.1)
                    sim.setJointTargetVelocity(motors["right"],  0.3)
                    print("align right.")
                elif imgR and not imgM and not imgL:
                    sim.setJointTargetVelocity(motors["left"], 0.3)
                    sim.setJointTargetVelocity(motors["right"], 0.1 )
                    print("align left.")
                elif imgL and imgM:
                    sim.setJointTargetVelocity(motors["left"],0.1 )
                    sim.setJointTargetVelocity(motors["right"], 0.2)
                    print("align slightly right.")
                elif imgR and imgM:
                    sim.setJointTargetVelocity(motors["left"], 0.2)
                    sim.setJointTargetVelocity(motors["right"], 0.1)
                    print("align slightly left.")
                time.sleep(0.01)
                imgL, imgM, imgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])
                found_line = not((imgL or imgR) and imgM)
            state = "LINE_FOLLOWING"
            print("Alignment complete, switching to line following.")
            continue


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

    fig.savefig("robot_path.png")
    cv2.destroyAllWindows()

def start_simulation():
    sim.startSimulation()
    time.sleep(1)
    execute()
    sim.stopSimulation()

if __name__ == '__main__':
    start_simulation()
