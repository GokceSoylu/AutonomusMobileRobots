from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import time
import keyboard
import cv2
import matplotlib.pyplot as plt

# Connect to CoppeliaSim
client = RemoteAPIClient()
sim = client.require('sim')

# Retrieve object handles
motors = {"left": sim.getObject('/DynamicLeftJoint'), "right": sim.getObject('/DynamicRightJoint')}
vision_sensor = sim.getObject('/Vision_sensor')
vehicle = sim.getObject('/LineTracer')

# Initial speed values
speed = 0.5
turn_factor = 0.3
rotate_speed = 0.5
px, py, vL_list, vR_list = [], [], [], []

# Update speed based on keyboard input
def update_speed():
    global speed
    if keyboard.is_pressed("+"):
        speed = min(2.0, speed + 0.5)
        print(f"Speed increased: {speed}")
    elif keyboard.is_pressed("-"):
        speed = max(0, speed - 0.5)
        print(f"Speed decreased: {speed}")

# Display camera image
def display_camera_image():
    img, res = sim.getVisionSensorImg(vision_sensor)
    if img and res:
        img_array = np.frombuffer(img, dtype=np.uint8).reshape((res[1], res[0], 3))
        img_array = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
        cv2.imshow("Camera Feed", img_array)
        cv2.waitKey(1)

# Control vehicle movement
def control_vehicle():
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 10))
    
    while sim.getSimulationState() != sim.simulation_stopped:
        update_speed()
        pos = sim.getObjectPosition(vehicle, -1)
        px.append(pos[0])
        py.append(pos[1])
        vL, vR = 0, 0
        
        if keyboard.is_pressed("w"):  # Forward
            vL, vR = speed, speed
        elif keyboard.is_pressed("s"):  # Backward
            vL, vR = -speed, -speed
        elif keyboard.is_pressed("a"):  # Turn left
            vL, vR = speed - turn_factor, speed + turn_factor
        elif keyboard.is_pressed("d"):  # Turn right
            vL, vR = speed + turn_factor, speed - turn_factor
        elif keyboard.is_pressed("q"):  # Rotate left in place
            vL, vR = -rotate_speed, rotate_speed
        elif keyboard.is_pressed("e"):  # Rotate right in place
            vL, vR = rotate_speed, -rotate_speed
        
        sim.setJointTargetVelocity(motors["left"], vL)
        sim.setJointTargetVelocity(motors["right"], vR)
        vL_list.append(vL)
        vR_list.append(vR)
        
        ax1.clear()
        ax1.scatter(py, px, color='blue', s=10)
        ax1.set_title("Vehicle Path")
        ax1.grid()
        
        ax2.clear()
        ax2.plot(vL_list, label="Left Motor", color='red')
        ax2.plot(vR_list, label="Right Motor", color='green')
        ax2.set_title("Motor Speed Changes")
        ax2.legend()
        ax2.grid()
        
        plt.pause(0.001)
        display_camera_image()
        
        time.sleep(0.05)
    
    plt.ioff()
    plt.show()
    cv2.destroyAllWindows()

# Start simulation
def start_simulation():
    sim.startSimulation()
    time.sleep(1)
    control_vehicle()
    sim.stopSimulation()

if __name__ == '__main__':
    start_simulation()
