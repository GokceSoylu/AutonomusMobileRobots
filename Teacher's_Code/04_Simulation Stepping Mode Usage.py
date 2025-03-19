# -*- coding: utf-8 -*-
"""
@author: acseckin
# Simulation Environment Settings:
# - CoppeliaSim must be running.
# - The remote API server must be enabled in CoppeliaSim.
"""


from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

# Create the remote API client and obtain the simulation module
client = RemoteAPIClient()
sim = client.require('sim')

# Enable stepping mode and start the simulation
sim.setStepping(True)
sim.startSimulation()

# Run the simulation for 5 steps and print the simulation time at each step
for i in range(5):
    sim.step()  # Advance one simulation step
    sim_time = sim.getSimulationTime()
    print(f"Simulation Step {i+1} - Simulation Time: {sim_time:.2f} s")
    time.sleep(1)  # Delay for observation

# Stop the simulation
sim.stopSimulation()

