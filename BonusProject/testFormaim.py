from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2
import time
client = RemoteAPIClient()
sim = client.require('sim')

left_motor = sim.getObjectHandle('/leftMotor')
right_motor = sim.getObjectHandle('/rightMotor')
vision_sensor = sim.getObjectHandle('/visionSensor')

vision_sensor = sim.getObjectHandle('/visionSensor')

sim.startSimulation()
time.sleep(1)

result, resolution, image = sim.getVisionSensorCharImage(vision_sensor)

print(f"Result: {result}")
if result == 0:
    print(f"Resolution: {resolution}")
    print(f"Image data length: {len(image)}")
else:
    print("Vision sensor image alınamadı. result != 0")
