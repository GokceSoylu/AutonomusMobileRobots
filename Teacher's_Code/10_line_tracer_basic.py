"""
Line Following Robot - Working Principle

This script connects to CoppeliaSim via the ZMQ Remote API and controls a line-following robot.

1. Initialization:
   - Establishes a connection to CoppeliaSim.
   - Retrieves object handles for sensors, motors, and the robot chassis.

2. Sensor Data Acquisition:
   - Uses infrared (IR) sensors to detect the black line on the surface.
   - Uses a vision sensor for capturing images and displaying them in real-time.
   - Uses a proximity sensor to detect obstacles in front of the robot.

3. Decision Making:
   - If none of the sensors detect the line, the robot moves forward.
   - If the left IR sensor detects the line, the robot turns left.
   - If the right IR sensor detects the line, the robot turns right.
   - If both sensors detect the line, the robot stops.
   - The sensor readings and corresponding motor speeds are printed in real-time for debugging.

4. Motor Control:
   - Adjusts the speed of the left and right motors based on sensor readings.
   - Uses setJointTargetVelocity to control movement.

5. Real-time Visualization:
   - Plots the robot’s movement on a graph using Matplotlib.
   - Displays the front camera feed using OpenCV.

6. Execution and Termination:
   - Runs in a loop until simulation stops.
   - Stops motors and closes all windows when the execution ends.

"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt


client = RemoteAPIClient()
sim = client.require('sim')

# Retrieve handles for necessary objects in CoppeliaSim
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

dmax = 0.35  # Max sensor distance
px, py = [], []

def get_image(sensor):
    """Capture an image from the vision sensor."""
    img, res = sim.getVisionSensorImg(sensor)
    if img:
        img = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return None

def sensor_ir(sensor):
    """Detect whether the infrared sensor sees the line."""
    img, _ = sim.getVisionSensorImg(sensor)
    return np.frombuffer(img, dtype=np.uint8)[0] > 40 if img else False

def get_distance(sensor):
    """Measure distance using the proximity sensor."""
    state, point, *_ = sim.readProximitySensor(sensor)
    return np.linalg.norm(point) if state else dmax

def execute():
    """Main control loop for line following."""
    plt.ion()
    while sim.getSimulationState() != sim.simulation_stopped:
        pos = sim.getObjectPosition(linetracer, -1)
        px.append(pos[0])
        py.append(pos[1])
        
        plt.clf()
        plt.scatter(py, px)
        plt.xlabel("X axis")
        plt.ylabel("Y axis")
        plt.grid()
        plt.show()
        plt.pause(0.0001)
        
        img = get_image(vision_sensor)
        if img is not None:
            cv2.imshow('Front Cam', img)
            if cv2.waitKey(5) & 0xFF == 27:
                break
        
        d = get_distance(proximity_sensor)
        v = 0.3  # Default speed
        imgM, imgL, imgR = sensor_ir(sensors["mid"]), sensor_ir(sensors["left"]), sensor_ir(sensors["right"])
        
        if not (imgM or imgL or imgR):
            wL, wR = v, v  # Move forward
            action = "Moving Forward"
        elif imgL and not imgR and not imgM:
            wL, wR = v, 0  # Turn left
            action = "Turning Left"
        elif imgR and not imgL and not imgM:
            wL, wR = 0, v  # Turn right
            action = "Turning Right"
        else:
            wL, wR = 0, 0  # Stop
            action = "Stopping"
        
        print(f"Sensors -> Left: {imgL}, Mid: {imgM}, Right: {imgR} | Motors -> Left Speed: {wL}, Right Speed: {wR} | Action: {action}")
        
        sim.setJointTargetVelocity(motors["left"], wL)
        sim.setJointTargetVelocity(motors["right"], wR)
    
    cv2.destroyAllWindows()

def start_simulation():
    """Start the simulation and execute the robot movement."""
    sim.startSimulation()
    time.sleep(1)
    execute()
    sim.stopSimulation()

if __name__ == '__main__':
    start_simulation()
