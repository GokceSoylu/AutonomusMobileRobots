# -*- coding: utf-8 -*-
"""
@author: acseckin

Simulation Environment Settings:
- CoppeliaSim must be running.
- The remote API server must be enabled in CoppeliaSim.
- The scene must contain an object named "Cuboid".
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Create the remote API client and obtain the simulation module
client = RemoteAPIClient()
sim = client.require('sim')

# Retrieve the handle for the object "Cuboid"
cuboid_handle = sim.getObjectHandle('Cuboid')

# Get the current position and orientation of the object
position = sim.getObjectPosition(cuboid_handle, -1)
orientation = sim.getObjectOrientation(cuboid_handle, -1)

print("Cuboid Handle:", cuboid_handle)
print("Position:", position)
print("Orientation:", orientation)
