from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2
import time

client = RemoteAPIClient()
sim = client.require('sim')

# === Obje İsimleri (CoppeliaSim sahnenizdeki isimlerle eşleşmeli) ===
robot_name = 'Body_transparent'
left_motor_name = 'DynamicLeftJoint'
right_motor_name = 'DynamicRightJoint'
aruco_sensor_name = 'Vision_sensor'

# === Object handles ===
robot_handle = -1
motors = {"left": -1, "right": -1}
aruco_sensor_handle = -1

# === Sabit Marker Pozisyonları ===
marker_positions = {
    0: np.array([1.500, 1.500, -0.0009]),
    42: np.array([-1.500, 1.500, -0.0009]),
    48: np.array([-1.500, -1.500, -0.0009]),
    6: np.array([1.500, -1.500, -0.0009]),
    45: np.array([-1.500, 0.000, -0.0009]),
    3: np.array([1.500, 0.000, -0.0009]),
    24: np.array([0.000, 0.000, -0.0009])
}

# === Yardımcı Fonksiyonlar ===
def get_object_by_name(name):
    handle = sim.getObject('/' + name)
    if handle == -1:
        handle = sim.getObject(name)
    return handle

def set_motor_velocities(left_vel, right_vel):
    sim.setJointTargetVelocity(motors["left"], left_vel)
    sim.setJointTargetVelocity(motors["right"], right_vel)

def get_robot_pose(robot_handle):
    position = sim.getObjectPosition(robot_handle, -1)
    orientation = sim.getObjectOrientation(robot_handle, -1)
    yaw = orientation[2]
    return np.array([position[0], position[1], yaw])

def navigate_to_marker(target_id, robot_handle, sensor_handle):
    angular_speed_factor = 1.5
    angle_threshold = 0.05
    linear_speed = 0.4
    angular_speed = 0.05
    distance_threshold = 0.1

    if target_id not in marker_positions:
        print(f"Hata: Hedef ID {target_id} için sabit konum bulunamadı.")
        return False

    target_pos = marker_positions[target_id][:2]

    while True:
        pose = get_robot_pose(robot_handle)
        dx, dy = target_pos[0] - pose[0], target_pos[1] - pose[1]
        angle_to_target = np.arctan2(dy, dx)
        angle_diff = (angle_to_target - pose[2] + np.pi) % (2 * np.pi) - np.pi
        distance = np.sqrt(dx ** 2 + dy ** 2)

        if abs(angle_diff) > angle_threshold:
            turn_speed = angular_speed * angle_diff * angular_speed_factor
            set_motor_velocities(-turn_speed, turn_speed)
        elif distance > distance_threshold:
            set_motor_velocities(linear_speed, linear_speed)
        else:
            set_motor_velocities(0, 0)
            print(f"Hedef {target_id}'ye ulaşıldı.")
            return True
        time.sleep(0.1)

def draw_letters(marker_list, robot_handle, sensor_handle):
    for marker_id in marker_list:
        print(f"İşaretçiye gidiliyor: {marker_id}")
        if navigate_to_marker(marker_id, robot_handle, sensor_handle):
            time.sleep(1)
    print("Harf çizimi tamamlandı.")
    set_motor_velocities(0, 0)

def navigate_student_id(marker_list, robot_handle, sensor_handle):
    for marker_id in marker_list:
        print(f"Öğrenci numarası işaretçisi: {marker_id}")
        if navigate_to_marker(marker_id, robot_handle, sensor_handle):
            time.sleep(1)
    print("Öğrenci numarası çizimi tamamlandı.")
    set_motor_velocities(0, 0)

# === Ana Çalıştırıcı Fonksiyon ===
def execute():
    global robot_handle, motors, aruco_sensor_handle

    # Handle'ları al
    robot_handle = get_object_by_name(robot_name)
    motors["left"] = get_object_by_name(left_motor_name)
    motors["right"] = get_object_by_name(right_motor_name)
    aruco_sensor_handle = get_object_by_name(aruco_sensor_name)

    if -1 in [robot_handle, motors["left"], motors["right"], aruco_sensor_handle]:
        print("Handle'lar alınamadı. İsimleri kontrol edin.")
        return

    # Görev tanımı
    target_markers_c = [0, 42, 48, 6, 24]
    target_markers_s = [0, 42, 45, 3, 6, 48, 24]
    target_markers_student = [24, 1, 8, 0, 5, 1, 1, 1, 24]

    draw_letters(target_markers_c, robot_handle, aruco_sensor_handle)
    time.sleep(5)
    draw_letters(target_markers_s, robot_handle, aruco_sensor_handle)
    time.sleep(5)
    navigate_student_id(target_markers_student, robot_handle, aruco_sensor_handle)

# === Simülasyonu Başlat ===
def start_simulation():
    sim.startSimulation()
    print("Simülasyon başlatılıyor...")
    while sim.getSimulationState() == sim.simulation_stopped:
        time.sleep(0.1)
    execute()

