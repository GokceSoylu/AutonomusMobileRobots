from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import matplotlib.pyplot as plt
import numpy as np

# ZMQ üzerinden bağlantı kur
client = RemoteAPIClient()
sim = client.get_object('sim')
print("Connected to CoppeliaSim via ZMQ.")

# Handle'ları al
left_motor = sim.getObject('/Pioneer_p3dx/leftMotor')
right_motor = sim.getObject('/Pioneer_p3dx/rightMotor')
robot = sim.getObject('/Pioneer_p3dx')
sensor = sim.getObject('/Pioneer_p3dx/ultrasonicSensor5')

# Simülasyonu başlat
sim.startSimulation()
time.sleep(0.5)

# PID parametreleri
Kp, Kd, Ki = 0.25, 0.02, 0.001
v_base = 1.0

# PID kontrol değişkenleri
last_error = 0
integral = 0

# FSM durumları
state = 'FOLLOW'

# Trajektori ve tur verisi
trajectory = []
lap_counter = 0
lap_start_time = time.time()
lap_times = []
passed_start = False
start_position = None
START_THRESHOLD = 0.5

def set_velocity(left, right):
    sim.setJointTargetVelocity(left_motor, left)
    sim.setJointTargetVelocity(right_motor, right)

def get_position():
    return sim.getObjectPosition(robot, -1)

def is_near_start(pos):
    if start_position is None:
        return False
    dx = pos[0] - start_position[0]
    dy = pos[1] - start_position[1]
    return (dx**2 + dy**2)**0.5 < START_THRESHOLD

def avoid_obstacle():
    global state
    state = 'AVOID'
    set_velocity(-0.5, 0.5)  # sağa dön
    time.sleep(0.8)
    set_velocity(1.0, 1.0)   # ileri
    time.sleep(0.8)
    state = 'FOLLOW'

print("Starting main loop...")

while True:
    position = get_position()
    trajectory.append((position[0], position[1]))

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

    # Sensör kontrolü
    result, detected, point, _, _ = sim.readProximitySensor(sensor)
    if detected:
        avoid_obstacle()
        continue

    # PID kontrol (şu an sabit hatayla)
    error = 0
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
sim.stopSimulation()
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
