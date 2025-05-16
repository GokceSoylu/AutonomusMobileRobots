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

forward_speed = 2.5
turn_speed = 2.5
search_speed = 2.0

mode = 'tracking'
last_seen_direction = 1
search_start_time = None
search_timeout = 100  # Arama süresi (saniye)

def get_image():
    image_data, resX, resY = sim.getVisionSensorCharImage(vision_sensor)
    img = np.frombuffer(image_data, dtype=np.uint8).reshape((resY, resX, 3))
    img = cv2.flip(img, 0)
    return img

def detect_target(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([80, 30, 30])
    upper_blue = np.array([140, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cx = None
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 50:
            M = cv2.moments(largest)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
    return cx, mask

try:
    while True:
        img = get_image()
        cx, mask = detect_target(img)

        if cx is not None:
            # Hedef tespit edildi
            if mode != 'tracking':
                print("[INFO] Hedef tekrar bulundu. Tracking moduna geçiliyor.")
            mode = 'tracking'
            img_center = img.shape[1] / 2
            error = cx - img_center
            last_seen_direction = 1 if error > 0 else -1

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
            # Hedef görünmüyor
            if mode != 'searching':
                print("[WARNING] Hedef kayboldu. Searching moduna geçiliyor.")
                mode = 'searching'
                search_start_time = time.time()
                # ÖNEMLİ: Aramaya geçerken motorları durdur ve momentum etkisini sıfırla
                sim.setJointTargetVelocity(left_motor, 0)
                sim.setJointTargetVelocity(right_motor, 0)
                time.sleep(0.2)  # Kısa duraklama
                
            elapsed = time.time() - search_start_time
            if elapsed < search_timeout:
                print("[SEARCHING] Hedef aranıyor... Robot sabit sola dönüyor.")
                sim.setJointTargetVelocity(left_motor, -search_speed)
                sim.setJointTargetVelocity(right_motor, search_speed)
            else:
                print("[IDLE] Hedef bulunamadı. Duruyor.")
                sim.setJointTargetVelocity(left_motor, 0)
                sim.setJointTargetVelocity(right_motor, 0)

        cv2.imshow('Camera', img)
        cv2.imshow('Mask', mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

sim.setJointTargetVelocity(left_motor, 0)
sim.setJointTargetVelocity(right_motor, 0)
sim.stopSimulation()
cv2.destroyAllWindows()
