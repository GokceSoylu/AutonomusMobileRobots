# -*- coding: utf-8 -*-
"""
@author: acseckin
# Simulation Environment Settings:
# - CoppeliaSim must be running.
# - The remote API server must be enabled in CoppeliaSim.
# - The scene must contain an object named "Cuboid" (or adjust the object name accordingly).

"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

# Create the remote API client and obtain the simulation module
client = RemoteAPIClient()
sim = client.require('sim')

# Retrieve the handle for the object "Cuboid"
cuboid_handle = sim.getObjectHandle('Cuboid')

# Enable stepping mode and start the simulation for real-time logging
sim.setStepping(True)
sim.startSimulation()

print("Starting real-time logging. Press Ctrl+C to stop.")
try:
    while True:
        sim.step()  # Advance one simulation step
        sim_time = sim.getSimulationTime()
        pos = sim.getObjectPosition(cuboid_handle, -1)
        ori = sim.getObjectOrientation(cuboid_handle, -1)
        print(f"Time: {sim_time:.2f} s, Position: {pos}, Orientation: {ori}")
        time.sleep(0.5)  # Log data every 0.5 seconds
except KeyboardInterrupt:
    print("Logging stopped by user.")

# Stop the simulation
sim.stopSimulation()
