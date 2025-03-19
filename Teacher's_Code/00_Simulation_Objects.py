# -*- coding: utf-8 -*-
"""
@author: acseckin
# Simulation Environment Settings:
# - CoppeliaSim must be running.
# - The remote API server must be enabled in CoppeliaSim.
# - The simulation environment should contain multiple objects.

Retrieve all objects in the scene.
sim.handle_all : for all objects in the simulation
sim.sceneobject_shape : for all shapes in the simulation
sim.sceneobject_camera
sim.sceneobject_proximitysensor
sim.sceneobject_visionsensor
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Create a connection to the remote API and obtain the simulation module
client = RemoteAPIClient()
sim = client.require('sim')

print ("All shapes:")
shape_list = sim.getObjectsInTree(sim.handle_scene, sim.sceneobject_shape, 0)
for i in shape_list:
    shape_name = sim.getObjectName(i)
    print(i, shape_name)
    
print ("All objects:")
objects_list = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
for i in objects_list:
    object_name = sim.getObjectName(i)
    print(i, object_name)