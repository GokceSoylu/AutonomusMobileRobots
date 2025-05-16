from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2
import time

client = RemoteAPIClient()
sim = client.require('sim')

vision_sensor_handle = sim.getObjectHandle('/visionSensor')

sim.startSimulation()
time.sleep(1)

result = sim.getVisionSensorCharImage(vision_sensor_handle)

print(f"Gelen veri tipi: {type(result)}, içerik uzunluğu: {len(result)}")

# result tuple olduğu için unpack ediyoruz
image_data, resX, resY = result

# numpy buffer'a çevir
img = np.frombuffer(image_data, dtype=np.uint8)
img = img.reshape(resY, resX, 3)
img = cv2.flip(img, 0)
img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

cv2.imshow('Vision Sensor', img)
cv2.waitKey(0)

sim.stopSimulation()
cv2.destroyAllWindows()
