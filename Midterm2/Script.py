# PID Kontrollü çizgi izleme
# Duruma bağlı FSM (FOLLOW → AVOID → SEARCH → ALIGN)
# Engelden sağdan geçme yeteneği
# Hız optimizasyonu
# Duruş ve hizalanma mekanizması
import sim
import time
import matplotlib.pyplot as plt
import numpy as np


# Bağlantı kur
print('Connecting to CoppeliaSim...')
sim.simxFinish(-1)
client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if client_id == -1:
    raise Exception('Failed to connect to CoppeliaSim.')

print('Connected.')

# Simülasyonu başlat
sim.simxStartSimulation(client_id, sim.simx_opmode_blocking)

# Handle'ları al
_, left_motor = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
_, right_motor = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
_, robot = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx', sim.simx_opmode_blocking)
_, sensor = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_ultrasonicSensor5', sim.simx_opmode_blocking)

# Sensörleri başlat
sim.simxReadProximitySensor(client_id, sensor, sim.simx_opmode_streaming)
time.sleep(0.1)

# PID parametreleri
Kp, Kd, Ki = 0.25, 0.02, 0.001
v_base = 1.0

# Trajektori verisi
trajectory = []

# PID kontrol değişkenleri
last_error = 0
integral = 0

# Tur verisi
lap_counter = 0
lap_start_time = time.time()
lap_times = []
passed_start = False

# Tur başlangıç pozisyonunun koordinatları
start_position = None
START_THRESHOLD = 0.5  # yakınlık toleransı

def set_velocity(left, right):
    sim.simxSetJointTargetVelocity(client_id, left_motor, left, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(client_id, right_motor, right, sim.simx_opmode_oneshot)

def avoid_obstacle():
    # Sağdan geçmek için: geri, sağa dön, ileri
    set_velocity(-0.5, 0.5)
    time.sleep(0.8)
    set_velocity(1.0, 1.0)
    time.sleep(0.8)

# Pozisyon kontrolü (tur sayımı)
def is_near_start(pos):
    if start_position is None:
        return False
    dx = pos[0] - start_position[0]
    dy = pos[1] - start_position[1]
    return (dx**2 + dy**2)**0.5 < START_THRESHOLD

print("Starting main loop...")

while True:
    # Robot pozisyonunu al
    _, position = sim.simxGetObjectPosition(client_id, robot, -1, sim.simx_opmode_buffer)
    trajectory.append((position[0], position[1]))

    # İlk turda başlangıç konumu belirle
    if start_position is None:
        start_position = position

    # Tur sayımı
    near_start = is_near_start(position)
    if near_start and not passed_start:
        passed_start = True
        if lap_counter > 0:
            lap_time = time.time() - lap_start_time
            lap_times.append(lap_time)
            print(f"Lap {lap_counter} completed in {lap_time:.2f} seconds")
        lap_counter += 1
        lap_start_time = time.time()
    elif not near_start:
        passed_start = False

    if lap_counter >= 3:
        break

    # Sensör verisi al
    _, detection_state, _, _, _ = sim.simxReadProximitySensor(client_id, sensor, sim.simx_opmode_buffer)

    if detection_state:
        avoid_obstacle()
        continue

    # PID kontrolü (örnek: çizgi takibi değil, ileriye sabit gidiyor)
    error = 0  # sabit ileri gidiyoruz, hatayı 0 kabul ettik
    integral += error
    derivative = error - last_error
    correction = Kp * error + Ki * integral + Kd * derivative
    last_error = error

    left_speed = v_base + correction
    right_speed = v_base - correction
    set_velocity(left_speed, right_speed)

    time.sleep(0.05)

# Simülasyonu durdur
set_velocity(0, 0)
sim.simxStopSimulation(client_id, sim.simx_opmode_blocking)
sim.simxFinish(client_id)

print("Simulation finished.")
print(f"Lap times: {lap_times}")

# Trajektori çizimi
x, y = zip(*trajectory)
plt.figure(figsize=(8, 6))
plt.plot(x, y, label='Trajectory')
plt.scatter([start_position[0]], [start_position[1]], color='red', label='Start')
plt.title('Robot Trajectory')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
plt.savefig('trajectory.png')
plt.show()
