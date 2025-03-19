# -*- coding: utf-8 -*-
"""
@author: acseckin
# Simulation Environment Settings:
# - CoppeliaSim must be running.
# - The remote API server must be enabled in CoppeliaSim.
# - The scene must contain an object named "Cuboid".
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math

# Create the remote API client and obtain the simulation module
client = RemoteAPIClient()
sim = client.require('sim')

# Retrieve the handle for the object "Cuboid"
cuboid_handle = sim.getObjectHandle('Cuboid')

# Get the current orientation (Euler angles) of the object
initial_orientation = sim.getObjectOrientation(cuboid_handle, -1)
print("Initial Orientation:", initial_orientation)

# Rotate the object around the Z-axis by 45 degrees (pi/4 radians)
new_orientation = list(initial_orientation)
new_orientation[2] += math.pi / 4  # Apply rotation around Z

sim.setObjectOrientation(cuboid_handle, -1, new_orientation)
print("Orientation after rotation:", sim.getObjectOrientation(cuboid_handle, -1))
