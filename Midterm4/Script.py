import time
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')
sim.stopSimulation()
time.sleep(1)
sim.startSimulation()

# Obje tanımlamaları
sensors = {
    'left': sim.getObject('/LineTracer/LeftSensor'),
    'middle': sim.getObject('/LineTracer/MiddleSensor'),
    'right': sim.getObject('/LineTracer/RightSensor'),
}
left_motor = sim.getObject('/LineTracer/LeftJoint')
right_motor = sim.getObject('/LineTracer/RightJoint')
proximity_sensor = sim.getObject('/LineTracer/Proximity_sensor')
robot_handle = sim.getObject('/LineTracer')

# PID parametreleri
Kp, Kd, Ki = 00.18, 0.015 ,  0.004
v_base = 1.0

# Yardımcı değişkenler
last_error = 0
integral = 0
lap = 0
obstacle_counter = 0 
positions = []
lap_start_time = time.time()
lap_times = []

def read_sensor(sensor_handle):#>
    result = sim.readVisionSensor(sensor_handle)
    if len(result) >= 2 and result[0] and result[1]:
        return result[1][0][0] < 0.5
    return False



def get_robot_position():
    pos = sim.getObjectPosition(robot_handle, -1)
    return pos[0], pos[1]

def avoid_obstacle_right():
    v_base = 2.0
    # Geri git
    sim.setJointTargetVelocity(left_motor, -v_base)
    sim.setJointTargetVelocity(right_motor, -v_base)
    time.sleep(0.8)

    # Sağa dön
    sim.setJointTargetVelocity(left_motor, v_base)
    sim.setJointTargetVelocity(right_motor, -v_base)
    time.sleep(1.0)

    # İleri git
    sim.setJointTargetVelocity(left_motor, v_base)
    sim.setJointTargetVelocity(right_motor, v_base)
    time.sleep(1.2)

    # Sola dön
    sim.setJointTargetVelocity(left_motor, -v_base)
    sim.setJointTargetVelocity(right_motor, v_base)
    time.sleep(0.9)

    # Düz git
    sim.setJointTargetVelocity(left_motor, v_base)
    sim.setJointTargetVelocity(right_motor, v_base)
    time.sleep(1.0)


def line_following_pid():
    global last_error, integral
    left = read_sensor(sensors['left'])
    middle = read_sensor(sensors['middle'])
    right = read_sensor(sensors['right'])

    # Hata hesabı: -1 (sol), 0 (orta), +1 (sağ)
    error = 0
    if left and not right:
        error = -1
    elif right and not left:
        error = 1
    elif not middle and not left and not right:
        error = last_error  # çizgiden çıkmış olabiliriz

    integral += error
    derivative = error - last_error
    correction = Kp * error + Ki * integral + Kd * derivative

    left_speed = v_base - correction
    right_speed = v_base + correction

    sim.setJointTargetVelocity(left_motor, left_speed)
    sim.setJointTargetVelocity(right_motor, right_speed)

    last_error = error

# Ana döngü
print("🚗 Başlıyor...")
start_time = time.time()
while sim.getSimulationState() != sim.simulation_stopped and lap < 2:
    # Engel kontrolü
    detected, _, _, _, _ = sim.readProximitySensor(proximity_sensor)
    if detected:
        print("⚠️ Engel algılandı, sağdan kaçınılıyor...")
        avoid_obstacle_right()
        obstacle_counter = 50  # yaklaşık 0.5 saniye PID devre dışı
        continue  # diğer kodları atla ve döngünün başına dön
    else:
        if obstacle_counter > 0:
            obstacle_counter -= 1
            continue  # hala kaçınma sonrası toparlanma süresinde
    line_following_pid()

    # Konum takibi
    x, y = get_robot_position()
    positions.append((x, y))

    # Tur kontrolü (başlangıç noktasına yakınlık)
    if abs(x) < 0.2 and abs(y) < 0.2 and time.time() - lap_start_time > 5:
        lap_time = time.time() - lap_start_time
        lap += 1
        lap_times.append(lap_time)
        print(f"🏁 Tur {lap} tamamlandı: {lap_time:.2f} saniye")
        lap_start_time = time.time()

    time.sleep(0.01)

# Simülasyonu durdur
sim.stopSimulation()
print("✅ Görev tamamlandı.")
for i, t in enumerate(lap_times):
    print(f"Tur {i+1}: {t:.2f} s")

# Grafik çizimi
x_vals = [p[0] for p in positions]
y_vals = [p[1] for p in positions]

plt.figure(figsize=(8, 6))
plt.plot(x_vals, y_vals, label="Robot Trajektorisi")
plt.scatter([0], [0], color="red", label="Başlangıç")
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Çizgi İzleyen Robot Trajektorisi")
plt.legend()
plt.grid()
plt.savefig("trajectory.png")
plt.show()
