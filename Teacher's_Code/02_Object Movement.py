# -*- coding: utf-8 -*-
"""
@author: acseckin
# Simulation Environment Settings:
# - CoppeliaSim must be running.
# - The remote API server must be enabled in CoppeliaSim.
# - The scene must contain an object named "Cuboid".
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

# Create the remote API client and obtain the simulation module
client = RemoteAPIClient()
sim = client.require('sim')

# Retrieve the handle for the object "Cuboid"
cuboid_handle = sim.getObjectHandle('Cuboid')

# Get the object's current position
initial_pos = sim.getObjectPosition(cuboid_handle, -1)
print("Initial Position:", initial_pos)

# Define new positions by modifying the X coordinate
pos_right = list(initial_pos)
pos_right[0] += 0.5  # Move right by 0.5 units

pos_left = list(initial_pos)
pos_left[0] -= 0.5  # Move left by 0.5 units

# Move the object to the right and print the updated position
sim.setObjectPosition(cuboid_handle, -1, pos_right)
print("Position after moving right:", sim.getObjectPosition(cuboid_handle, -1))
time.sleep(1)

# Move the object to the left and print the updated position
sim.setObjectPosition(cuboid_handle, -1, pos_left)
print("Position after moving left:", sim.getObjectPosition(cuboid_handle, -1))
