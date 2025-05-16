from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2
import time

client = RemoteAPIClient()
sim = client.require('sim')

left_motor = sim.getObjectHandle('/leftMotor')
right_motor = sim.getObjectHandle('/rightMotor')
vision_sensor = sim.getObjectHandle('/visionSensor')

sim.startSimulation()
time.sleep(1)

forward_speed = 2.0
turn_speed = 1.5

def get_image():
    result, resolution, image = sim.getVisionSensorCharImage(vision_sensor)
    if result != 0:
        print("Vision sensor image alınamadı.")
        return None
    img = np.frombuffer(image, dtype=np.uint8).reshape((resolution[1], resolution[0], 3))
    img = cv2.flip(img, 0)
    return img

def process_image(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 | mask2

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            return cx, mask
    return None, mask

try:
    while True:
        img = get_image()
        if img is None:
            sim.setJointTargetVelocity(left_motor, 0)
            sim.setJointTargetVelocity(right_motor, 0)
            break

        cx, mask = process_image(img)

        if cx is not None:
            img_center = img.shape[1] / 2
            error = cx - img_center

            if abs(error) < 20:
                sim.setJointTargetVelocity(left_motor, forward_speed)
                sim.setJointTargetVelocity(right_motor, forward_speed)
            elif error > 0:
                sim.setJointTargetVelocity(left_motor, forward_speed)
                sim.setJointTargetVelocity(right_motor, forward_speed - turn_speed)
            else:
                sim.setJointTargetVelocity(left_motor, forward_speed - turn_speed)
                sim.setJointTargetVelocity(right_motor, forward_speed)
        else:
            sim.setJointTargetVelocity(left_motor, 0)
            sim.setJointTargetVelocity(right_motor, 0)

        cv2.imshow('Vision Sensor', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        cv2.imshow('Mask', mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

sim.setJointTargetVelocity(left_motor, 0)
sim.setJointTargetVelocity(right_motor, 0)
sim.stopSimulation()
cv2.destroyAllWindows()

