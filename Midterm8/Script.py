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
Kp, Kd, Ki = 0.22, 0.02, 0.006  # Ayarlanmış PID parametreleri
e_old = 0
e_integral = 0
v_base = 0.7  # Ayarlanmış temel hız
d_obstacle = 0.30
obstacle_detected_time = 0
lap_count = 0
lap_start_time = 0
lap_times = []
max_laps = 2  # İzlenecek maksimum tur sayısı

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

def is_on_line(imgL, imgM, imgR):
    return imgL or imgM or imgR

def execute():
    global e_old, e_integral, obstacle_detected_time, lap_count, lap_start_time, lap_times, max_laps
    state = "FOLLOWING"
    vL_list, vR_list, px, py = [], [], [], []

    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1)

    sim_start_time = sim.getSimulationTime()

    while sim.getSimulationState() != sim.simulation_stopped and lap_count < max_laps:
        current_time = sim.getSimulationTime()
        pos = sim.getObjectPosition(linetracer, -1)
        if pos:
            px.append(pos[0])
            py.append(pos[1])

            d = get_distance(proximity_sensor)
            imgL, imgM, imgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])

            # Lap tamamlama kontrolü (basit bir yaklaşımla X koordinatına göre)
            lap_threshold = 1.0
            if pos[0] > lap_threshold and state != "AVOIDANCE":
                if lap_count == 0:
                    lap_count = 1
                    lap_time = current_time - sim_start_time
                    lap_times.append(lap_time)
                    print(f"1. Tur Tamamlama Süresi: {lap_time:.2f} saniye")
                    lap_start_time = current_time
                elif lap_count == 1 and current_time - lap_start_time > 5:
                    lap_count = 2
                    lap_time = current_time - lap_start_time
                    lap_times.append(lap_time)
                    print(f"2. Tur Tamamlama Süresi: {lap_time:.2f} saniye")
                    break  # İki tur tamamlandı, döngüyü sonlandır

            if state == "FOLLOWING":
                if d < d_obstacle:
                    print("FOLLOWING: Obstacle detected")
                    state = "AVOIDANCE"
                    obstacle_detected_time = current_time
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
                avoidance_start_time = current_time
                turned = False
                while current_time - avoidance_start_time < 1.5:  # Sağa dönüş süresi artırıldı
                    print("AVOIDANCE: turning right")
                    sim.setJointTargetVelocity(motors["left"], 0.5)
                    sim.setJointTargetVelocity(motors["right"], -0.5)
                    imgL, imgM, imgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])
                    if not is_on_line(imgL, imgM, imgR):
                        turned = True
                    current_time = sim.getSimulationTime()
                    if turned and current_time - avoidance_start_time > 0.5: # Biraz dönüp sonra ileri gitmeye başla
                        break

                forward_start_time = current_time
                moved_forward = False
                while current_time - forward_start_time < 1.5: # İleri gitme süresi
                    print("AVOIDANCE: moving forward after right turn.")
                    sim.setJointTargetVelocity(motors["left"], 0.6)
                    sim.setJointTargetVelocity(motors["right"], 0.6)
                    imgL, imgM, imgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])
                    if not is_on_line(imgL, imgM, imgR):
                        moved_forward = True
                    current_time = sim.getSimulationTime()
                    if moved_forward and current_time - forward_start_time > 0.5:
                        break

                search_start_time = current_time
                while current_time - search_start_time < 3: # Çizgiyi bulmak için arama süresi
                    print("AVOIDANCE: searching line by turning left")
                    sim.setJointTargetVelocity(motors["left"], -0.4)
                    sim.setJointTargetVelocity(motors["right"], 0.4)
                    imgL, imgM, imgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])
                    if is_on_line(imgL, imgM, imgR):
                        print("AVOIDANCE: Line found after avoidance")
                        state = "FOLLOWING"
                        e_integral = 0 # İntegrali sıfırla
                        e_old = 0      # Önceki hatayı sıfırla
                        break
                    current_time = sim.getSimulationTime()

                if state == "AVOIDANCE": # Eğer hala çizgiyi bulamadıysa daha geniş arama
                    search_start_time = current_time
                    while current_time - search_start_time < 5:
                        print("AVOIDANCE: wider search turning right")
                        sim.setJointTargetVelocity(motors["left"], 0.4)
                        sim.setJointTargetVelocity(motors["right"], -0.4)
                        imgL, imgM, imgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])
                        if is_on_line(imgL, imgM, imgR):
                            print("AVOIDANCE: Line found after wider search")
                            state = "FOLLOWING"
                            e_integral = 0
                            e_old = 0
                            break
                        current_time = sim.getSimulationTime()
                    if state == "AVOIDANCE":
                        print("AVOIDANCE: Line not found, going back to search")
                        state = "SEARCH" # Son çare olarak SEARCH durumuna geri dön

            elif state == "SEARCH":
                search_duration = 5
                search_start_time = current_time
                while current_time - search_start_time < search_duration:
                    d = get_distance(proximity_sensor)
                    imgL, imgM, imgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])
                    found_line = is_on_line(imgL, imgM, imgR)
                    print("SEARCH:", d, found_line, current_time - search_start_time)
                    sim.setJointTargetVelocity(motors["left"], -0.3) # Sağa doğru daha yavaş arama
                    sim.setJointTargetVelocity(motors["right"], 0.3)

                    if d < d_obstacle:
                        print("SEARCH: Obstacle detected")
                        state = "AVOIDANCE"
                        obstacle_detected_time = current_time
                        break
                    elif found_line:
                        print("SEARCH: Line found")
                        state = "ALIGNMENT"
                        break
                    current_time = sim.getSimulationTime()
                if state == "SEARCH": # Eğer arama süresi dolduysa ve çizgi bulunamadıysa
                    print("SEARCH: Line not found after search, trying wider turn")
                    state = "AVOIDANCE" # Tekrar engelden kaçınma rutinine git

            elif state == "ALIGNMENT":
                alignment_start_time = current_time
                while current_time - alignment_start_time < 2: # Hizalama süresi
                    fimgL, fimgM, fimgR = sensor_ir(sensors["left"]), sensor_ir(sensors["mid"]), sensor_ir(sensors["right"])
                    if not fimgM: # Orta sensör çizgiyi görmüyorsa hizala
                        if fimgL and not fimgR:
                            sim.setJointTargetVelocity(motors["left"], 0.1)
                            sim.setJointTargetVelocity(motors["right"], -0.1)
                        elif fimgR and not fimgL:
                            sim.setJointTargetVelocity(motors["left"], -0.1)
                            sim.setJointTargetVelocity(motors["right"], 0.1)
                        elif not fimgL and not fimgR:
                            sim.setJointTargetVelocity(motors["left"], 0.2)
                            sim.setJointTargetVelocity(motors["right"], -0.2) # Hafif sola dön
                    else:
                        print("ALIGNMENT: Aligned")
                        state = "FOLLOWING"
                        break
                    current_time = sim.getSimulationTime()
                if state == "ALIGNMENT":
                    print("ALIGNMENT: Alignment failed, going back to following")
                    state = "FOLLOWING" # Hizalama başarısız olursa takibe geri dön

            img = get_image(vision_sensor)
            if img is not None:
                cv2.imshow('Front Cam', img)
                if cv2.waitKey(5) & 0xFF == 27:
                    break

            ax1.clear()
            ax1.scatter(py, px, s=5)
            ax1.set_xlabel("X axis")
            ax1.set_ylabel("Y axis")
            ax1.set_title("Robot Path")
            ax1.grid()
            ax1.set_aspect('equal', adjustable='box') # Eşit ölçeklendirme

            ax2.clear()
            ax2.plot(vL_list, label="Left Motor")
            ax2.plot(vR_list, label="Right Motor")
            ax2.set_title("Motor Speeds")
            ax2.legend()
            ax2.grid()

            plt.pause(0.0001)

    cv2.destroyAllWindows()
    plt.savefig("robot_trajectory.png") # Yörüngeyi kaydet

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