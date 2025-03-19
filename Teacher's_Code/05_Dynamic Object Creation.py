# -*- coding: utf-8 -*-
"""
@author: acseckin
# Simulation Environment Settings:
# - CoppeliaSim must be running.
# - The remote API server must be enabled in CoppeliaSim.
# - Ensure that dynamic object creation is allowed in the simulation.
"""



from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import time

# Create the remote API client and obtain the simulation module
client = RemoteAPIClient()
sim = client.require('sim')

# Create a cone primitive shape using sim.createPureShape
# Shape type 3 corresponds to a cone.
# The 'sizes' parameter is [base diameter, base diameter, height]
cone_handle = sim.createPureShape(3, 0, [1.0, 1.0, 2.0], 1.0, None)
print("Created cone with handle:", cone_handle)

# Advance one simulation step to process the creation
sim.step()

# Get and print the initial position and orientation of the cone
position = sim.getObjectPosition(cone_handle, -1)
orientation = sim.getObjectOrientation(cone_handle, -1)
print("Cone Initial Position:", position)
print("Cone Initial Orientation:", orientation)

# Move the cone to the right by modifying its X coordinate
pos_right = list(position)
pos_right[0] += 0.5  # Move right
sim.setObjectPosition(cone_handle, -1, pos_right)
sim.step()
print("Cone Position after moving right:", sim.getObjectPosition(cone_handle, -1))
time.sleep(1)

# Move the cone to the left by modifying its X coordinate
pos_left = list(position)
pos_left[0] -= 0.5  # Move left
sim.setObjectPosition(cone_handle, -1, pos_left)
sim.step()
print("Cone Position after moving left:", sim.getObjectPosition(cone_handle, -1))
time.sleep(1)

# Rotate the cone around the Z-axis by 45 degrees (pi/4 radians)
new_orientation = list(orientation)
new_orientation[2] += math.pi / 4
sim.setObjectOrientation(cone_handle, -1, new_orientation)
sim.step()
print("Cone Orientation after rotation:", sim.getObjectOrientation(cone_handle, -1))
