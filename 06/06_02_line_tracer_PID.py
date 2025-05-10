from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt

"""
PID Line Following Robot - ZMQ Remote API Version

This script implements a PID-controlled line-following robot using the CoppeliaSim ZMQ Remote API.

1. **Initialization:**
   - Connects to CoppeliaSim via ZMQ Remote API.
   - Retrieves object handles for sensors, motors, and robot chassis.

2. **Sensor Data Acquisition:**
   - Uses IR sensors to detect the black line.
   - Captures front camera images for visualization.
   - Uses a proximity sensor to detect obstacles.

3. **Enhanced PID Control for Steering:**
   - Computes error values based on sensor inputs.
   - Adjusts motor speeds dynamically using PID correction with adaptive gains:
     
     \[ \text{correction} = K_p \cdot e + K_d \cdot (e - e_{old}) + K_i \cdot \sum e \]
     
   - Adapts gains based on aggressive turns to prevent instability.
   - Ensures smoother and more responsive turns.

4. **Motor Control:**
   - Adjusts left and right motor speeds using calculated PID corrections.

5. **Visualization & Debugging:**
   - Plots real-time robot path.
   - Displays front camera feed.
   - Prints sensor readings, motor speeds, and PID values for debugging.
   - Plots real-time motor speed changes.

"""

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
px, py, vL_list, vR_list = [], [], [], []

# PID coefficients with adaptive tuning
Kp, Kd, Ki = 0.05, 0.02, 0.002  # Increased gains for better response
e_old = 0
e_integral = 0

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
    """Main control loop with enhanced PID steering."""
    global e_old, e_integral
    plt.ion()
    v = 0.3  # Base speed
    fig, (ax1, ax2) = plt.subplots(2, 1)
    
    while sim.getSimulationState() != sim.simulation_stopped:
        pos = sim.getObjectPosition(linetracer, -1)
        px.append(pos[0])
        py.append(pos[1])
        
        img = get_image(vision_sensor)
        if img is not None:
            cv2.imshow('Front Cam', img)
            if cv2.waitKey(5) & 0xFF == 27:
                break
        
        d = get_distance(proximity_sensor)
        imgM, imgL, imgR = sensor_ir(sensors["mid"]), sensor_ir(sensors["left"]), sensor_ir(sensors["right"])
        
        # Compute error
        if imgL and not imgR and not imgM:
            e_new = -1  # Left deviation
        elif imgR and not imgL and not imgM:
            e_new = 1  # Right deviation
        else:
            e_new = 0  # Centered or lost line
        
        # Adaptive gain adjustment for sharp turns
        if abs(e_new) > 0:
            adaptive_Kp = Kp * 1.5
            adaptive_Kd = Kd * 1.5
        else:
            adaptive_Kp = Kp
            adaptive_Kd = Kd
        
        # PID correction
        e_integral += e_new  # Accumulate integral error
        correction = adaptive_Kp * e_new + adaptive_Kd * (e_new - e_old) + Ki * e_integral
        e_old = e_new
        
        # Adjust motor speeds
        vL = v - correction
        vR = v + correction
        vL_list.append(vL)
        vR_list.append(vR)
        
        print(f"Sensors -> Left: {imgL}, Mid: {imgM}, Right: {imgR} | Error: {e_new} | Correction: {correction:.4f} | Motors -> Left Speed: {vL:.4f}, Right Speed: {vR:.4f}")
        
        sim.setJointTargetVelocity(motors["left"], vL)
        sim.setJointTargetVelocity(motors["right"], vR)
        
        # Update plots without recreating them
        ax1.clear()
        ax1.scatter(py, px)
        ax1.set_xlabel("X axis")
        ax1.set_ylabel("Y axis")
        ax1.set_title("Robot Path")
        ax1.grid()
        
        ax2.clear()
        ax2.plot(vL_list, label="Left Motor Speed")
        ax2.plot(vR_list, label="Right Motor Speed")
        ax2.set_xlabel("Time Step")
        ax2.set_ylabel("Speed")
        ax2.set_title("Motor Speed Changes")
        ax2.legend()
        ax2.grid()
        
        plt.pause(0.0001)
    
    cv2.destroyAllWindows()

def start_simulation():
    """Start the simulation and execute the robot movement."""
    sim.startSimulation()
    time.sleep(1)
    execute()
    sim.stopSimulation()

if __name__ == '__main__':
    start_simulation()