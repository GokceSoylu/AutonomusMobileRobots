# -*- coding: utf-8 -*-
"""
@author: acseckin
# Simulation Environment Settings:
# - CoppeliaSim must be running.
# - The remote API server must be enabled in CoppeliaSim.
# - The scene should contain an object named "Cuboid". This example also demonstrates error handling for a non-existent object.

"""


from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Create the remote API client and obtain the simulation module
client = RemoteAPIClient()
sim = client.require('sim')

# Function to safely retrieve an object handle
def safe_get_object_handle(object_name):
    try:
        handle = sim.getObjectHandle(object_name)
        print(f"Successfully retrieved handle for '{object_name}': {handle}")
        return handle
    except Exception as e:
        print(f"Error retrieving handle for '{object_name}': {e}")
        return None

# Attempt to retrieve a valid object handle
cuboid_handle = safe_get_object_handle('Cuboid')

# Attempt to retrieve a non-existent object handle to demonstrate error handling
invalid_handle = safe_get_object_handle('NonExistentObject')

# Continue with further operations only if cuboid_handle is valid
if cuboid_handle is not None:
    pos = sim.getObjectPosition(cuboid_handle, -1)
    print("Cuboid Position:", pos)
else:
    print("Cuboid handle not found. Cannot proceed with position reading.")
