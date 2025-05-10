from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

client = RemoteAPIClient()
sim = client.getObject('sim')

print("✅ Connected.")

# Simülasyonu başlat
sim.startSimulation()

# Motorları (eklemleri) al
left_motor = sim.getObject('/LineTracer/DynamicLeftJoint')
right_motor = sim.getObject('/LineTracer/DynamicRightJoint')

# İleri doğru hareket için her iki motora pozitif hız ver
sim.setJointTargetVelocity(left_motor, 2.0)
sim.setJointTargetVelocity(right_motor, 2.0)

# 5 saniye bekle
time.sleep(5)

# Hızları sıfırla (dursun)
sim.setJointTargetVelocity(left_motor, 0.0)
sim.setJointTargetVelocity(right_motor, 0.0)

# Simülasyonu durdur
sim.stopSimulation()
