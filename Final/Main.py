from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
import numpy as np

client = RemoteAPIClient()
sim = client.require('sim')

# === Obje İsimleri (CoppeliaSim sahnenizdeki isimlerle eşleşmeli) ===
robot_name = 'Body_transparent'  # Robotun ana objesinin adı
left_motor_name = 'DynamicLeftJoint' # Sol motor joint'inin adı
right_motor_name = 'DynamicRightJoint' # Sağ motor joint'inin adı
aruco_sensor_name = 'Vision_sensor' # ArUco sensörünün (kamera) adı

# === Object handles ===
robot_handle = -1
motors = {"left": -1, "right": -1}
aruco_sensor_handle = -1

# === Parametreler ===
linear_speed = 0.2
angular_speed = 0.1
distance_threshold = 0.1

def get_object_by_name(name):
    """İsme göre objeyi bulur."""
    handle = sim.getObject('/' + name)
    if handle == -1:
        handle = sim.getObject(name)
    return handle

def set_motor_velocities(left_vel, right_vel):
    sim.setJointTargetVelocity(motors["left"], left_vel)
    sim.setJointTargetVelocity(motors["right"], right_vel)

def get_detected_markers(sensor_handle):
    """Algılanan ArUco işaretçilerinin ID'lerini ve pozisyonlarını döndürür."""
    detection_result = sim.readVisionSensor(sensor_handle)
    print(f"sim.readVisionSensor() çıktısı: {detection_result}")
    if detection_result == -1:
        print("Hata: Vizyon sensöründen veri okunamadı.")
        return []
    elif detection_result and detection_result[0] == 0:  # sim.simulation_ok yerine 0 kontrolü
        detected_objects_data = detection_result[2]
        markers = []
        # 'detected_objects_data' listesini ikili gruplar halinde işle
        for i in range(0, len(detected_objects_data), 2):
            if len(detected_objects_data[i]) > 0 and detected_objects_data[i][0] == sim.object_aruco_marker:
                marker_id = detected_objects_data[i][1]
                # Pozisyon bilgisi bir sonraki eleman (index i+1) içinde olmalı
                if i + 1 < len(detected_objects_data):
                    marker_pose = detected_objects_data[i + 1]
                    markers.append({'id': marker_id, 'pose': marker_pose})
        return markers
    else:
        print(f"Uyarı: Beklenmeyen sim.readVisionSensor() sonucu: {detection_result}")
        return []

def get_robot_pose(robot_handle):
    """Robotun pozisyonunu (x, y, alpha) döndürür."""
    robot_position = sim.getObjectPosition(robot_handle, -1)
    robot_orientation = sim.getObjectOrientation(robot_handle, -1)
    return np.array([robot_position[0], robot_position[1], robot_orientation[2]])

def navigate_to_marker(target_id, robot_handle, sensor_handle):
    """Belirli bir ID'ye sahip işaretçiye doğru robotu hareket ettirir."""
    angular_speed_factor = 0.5  # Dönüş hızını kontrol etmek için bir faktör
    while True:
        detected_markers = get_detected_markers(sensor_handle)
        robot_pose = get_robot_pose(robot_handle)

        target_marker = None
        for marker in detected_markers:
            if marker['id'] == target_id:
                target_marker = marker
                break

        if target_marker:
            # Hedefin robot referansına göre göreli pozisyonu
            dx = target_marker['pose'][0]
            dy = target_marker['pose'][1]
            distance_to_target = np.sqrt(dx**2 + dy**2)
            angle_to_target = np.arctan2(dy, dx)
            robot_orientation = robot_pose[2]
            angle_difference = angle_to_target - robot_orientation
            angle_difference = (angle_difference + np.pi) % (2 * np.pi) - np.pi # -pi ile pi arasına normalleştir

            if abs(angle_difference) > 0.1:  # Hedefe dön
                turn_speed = angular_speed * angle_difference * angular_speed_factor
                set_motor_velocities(-turn_speed, turn_speed)
                print(f"Dönüyor. Açı farkı: {angle_difference:.2f}")
            elif distance_to_target > distance_threshold:  # Hedefe git
                set_motor_velocities(linear_speed, linear_speed)
                print(f"İlerliyor. Mesafe: {distance_to_target:.2f}")
            else:
                set_motor_velocities(0, 0)
                print(f"Hedef işaretçi {target_id}'ye ulaşıldı.")
                return True
            time.sleep(0.1)
        else:
            # Eğer hedef işaretçi algılanmadıysa, dönmeye devam et (çizgiyi arama davranışı eklenebilir)
            set_motor_velocities(-angular_speed * 0.2, angular_speed * 0.2) # Daha yavaş dön
            print(f"Hedef {target_id} algılanamadı, dönüyor...")
            time.sleep(0.1)

def draw_letters(marker_list, robot_handle, sensor_handle):
    current_target_index = 0
    print(f"Harf çizimi başlatılıyor. Hedefler: {marker_list}")
    while current_target_index < len(marker_list):
        target_marker_id = marker_list[current_target_index]
        print(f"Şu anki hedef: İşaretçi {target_marker_id}")
        if navigate_to_marker(target_marker_id, robot_handle, sensor_handle):
            current_target_index += 1
        time.sleep(1)
    print("Harf çizimi tamamlandı.")
    set_motor_velocities(0, 0)

def navigate_student_id(marker_list, robot_handle, sensor_handle):
    current_target_index = 0
    print(f"Öğrenci numarasına göre navigasyon başlatılıyor. Hedefler: {marker_list}")
    while current_target_index < len(marker_list):
        target_marker_id = marker_list[current_target_index]
        print(f"Şu anki hedef: İşaretçi {target_marker_id}")
        detected = False
        for marker in get_detected_markers(sensor_handle):
            if marker['id'] == target_marker_id:
                detected = True
                break
        if detected:
            print(f"İşaretçi {target_marker_id} algılandı.")
            current_target_index += 1
        else:
            print(f"Uyarı: İşaretçi {target_marker_id} algılanamadı.")
            time.sleep(2)
        time.sleep(1)
    print("Öğrenci numarasına göre navigasyon tamamlandı.")
    set_motor_velocities(0, 0)

def execute():
    global robot_handle, motors, aruco_sensor_handle

    # İsimlere göre handlları al
    robot_handle = get_object_by_name(robot_name)
    motors["left"] = get_object_by_name(left_motor_name)
    motors["right"] = get_object_by_name(right_motor_name)
    aruco_sensor_handle = get_object_by_name(aruco_sensor_name)

    if robot_handle == -1 or motors["left"] == -1 or motors["right"] == -1 or aruco_sensor_handle == -1:
        print("Hata: Robot, motorlar veya ArUco sensörü isimlere göre bulunamadı. İsimleri kontrol edin.")
        print(f"Robot Handle: {robot_handle}")
        print(f"Sol Motor Handle: {motors['left']}")
        print(f"Sağ Motor Handle: {motors['right']}")
        print(f"ArUco Kamera Handle: {aruco_sensor_handle}")
        return

    # Görev tanımları
    target_markers_c = [24, 0, 42, 48, 6]
    target_markers_s = [24, 0, 42, 45, 3, 6, 48]
    target_markers_student = [24, 24]

    # Görev sırası
    draw_letters(target_markers_c, robot_handle, aruco_sensor_handle)
    time.sleep(5)
    draw_letters(target_markers_s, robot_handle, aruco_sensor_handle)
    time.sleep(5)
    navigate_student_id(target_markers_student, robot_handle, aruco_sensor_handle)

def start_simulation():
    sim.startSimulation()
    print("Simülasyon başlatılıyor...")
    while sim.getSimulationState() == sim.simulation_stopped:
        print("Bekleniyor: sim.getSimulationState() == sim.simulation_stopped")
        time.sleep(0.1)
    execute()
    sim.stopSimulation()

if __name__ == '__main__':
    start_simulation()