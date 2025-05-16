from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2
import numpy as np
import time

client = RemoteAPIClient()
sim = client.require('sim')

vision_sensor_handle = sim.getObjectHandle('/visionSensor')

sim.startSimulation()

time.sleep(1)

# 3 değer dönecek şekilde doğru unpack
result, image, resolution = sim.getVisionSensorCharImage(vision_sensor_handle)

#print(f"Result: {result}, Image type: {type(image)}, Resolution: {resolution}")

if result == 1:
    resX, resY = resolution
    img = np.frombuffer(image, dtype=np.uint8)
    img = img.reshape(resY, resX, 3)
    img = cv2.flip(img, 0)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    cv2.imshow('Vision Sensor', img)
    cv2.waitKey(0)
else:
    print("Görüntü alınamadı.")

sim.stopSimulation()
cv2.destroyAllWindows()
