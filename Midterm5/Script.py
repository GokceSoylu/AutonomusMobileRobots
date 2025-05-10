from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
# Yukarıdaki importlar ve nesne handle alma kısımları değişmeden devam ediyor...
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

# Yeni PID ve hız ayarları:
dmax = 0.35
Kp, Kd, Ki = 0.18, 0.015 ,  0.004
e_old = 0
e_integral = 0
v_base = 0.8
d_obstacle = 0.30

# Tur takibi için değişkenler
lap_count = 2
lap_times = []
lap_start_time = None
start_pos = None
lap_threshold = 0.4  # start pozisyonuna geri dönüldüğünü algılamak için

def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))
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
    global e_old, e_integral, lap_start_time, start_pos

    state = "FOLLOWING"
    vL_list, vR_list, px, py = [], [], [], []
    current_lap = 0

    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1)

    while sim.getSimulationState() != sim.simulation_stopped:
        pos = sim.getObjectPosition(linetracer, -1)
        px.append(pos[0])
        py.append(pos[1])

        # Lap kontrolü
        if current_lap < lap_count:
            if start_pos is None:
                start_pos = pos
                lap_start_time = time.time()
            else:
                if distance(pos, start_pos) < lap_threshold and len(px) > 50:
                    lap_time = time.time() - lap_start_time
                    lap_times.append(lap_time)
                    print(f"🏁 Lap {current_lap+1} completed in {lap_time:.2f} seconds")
                    current_lap += 1
                    lap_start_time = time.time()

        d = get_distance(proximity_sensor)
        imgL, imgM, imgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])

        if state == "FOLLOWING":
            if d < d_obstacle:
                print("FOLLOWING: Obstacle detected")
                state = "AVOIDANCE"
                continue

            # PID Error hesabı
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

        elif state == "AVOIDANCE":
            # Sağdan dolan: geri -> sağa dön -> ileri
            print("AVOIDANCE: Reversing")
            sim.setJointTargetVelocity(motors["left"], -v_base)
            sim.setJointTargetVelocity(motors["right"], -v_base)
            time.sleep(0.8)

            print("AVOIDANCE: Turning right")
            sim.setJointTargetVelocity(motors["left"], v_base)
            sim.setJointTargetVelocity(motors["right"], -v_base)
            time.sleep(1.0)

            print("AVOIDANCE: Moving forward")
            sim.setJointTargetVelocity(motors["left"], v_base)
            sim.setJointTargetVelocity(motors["right"], v_base)
            time.sleep(1.2)

            print("AVOIDANCE: Aligning back")
            sim.setJointTargetVelocity(motors["left"], -v_base)
            sim.setJointTargetVelocity(motors["right"], v_base)
            time.sleep(1.0)

            state = "FOLLOWING"

        # Görüntüleme
        img = get_image(vision_sensor)
        if img is not None:
            cv2.imshow('Front Cam', img)
            if cv2.waitKey(5) & 0xFF == 27:
                break

        ax1.clear()
        ax1.scatter(py, px, s=1)
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

        if current_lap == lap_count:
            print("✅ All laps completed.")
            break

    # Save trajectory plot
    plt.ioff()
    fig.savefig("robot_path.png")
    print("📊 Trajectory plot saved as 'robot_path.png'.")

    cv2.destroyAllWindows()

def start_simulation():
    sim.startSimulation()
    print("Starting simulation...")
    while sim.getSimulationState() == sim.simulation_stopped:
        time.sleep(0.1)
    execute()
    sim.stopSimulation()

if __name__ == '__main__':
    start_simulation()
