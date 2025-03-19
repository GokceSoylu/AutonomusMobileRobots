from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import time

# Create a connection to the remote API and obtain the simulation module
client = RemoteAPIClient()
sim = client.require('sim')

# Enable stepping mode and start the simulation
sim.setStepping(True)
sim.startSimulation()

# --- Create a Cone Primitive Shape ---
# Use the function sim.createPureShape to create a cone.
# Parameters:
#   shape type: 3 (for cone)
#   options: 0 (default options)
#   sizes: [base diameter, base diameter, height]
#   mass: 1.0 (for example)
#   appearance: None (default appearance)
cone_handle = sim.createPureShape(3, 0, [1.0, 1.0, 2.0], 1.0, None)
print("Created cone with handle:", cone_handle)

# Advance one simulation step to ensure the creation is processed
sim.step()

# --- Get and Print Initial Position ---
initial_pos = sim.getObjectPosition(cone_handle, -1)
print("Initial Cone Position:", initial_pos)

# Define new positions by adjusting the X coordinate (assuming the default frame is used)
pos_left = list(initial_pos)
pos_left[0] -= 0.5  # Move 0.5 units to the left
pos_right = list(initial_pos)
pos_right[0] += 0.5  # Move 0.5 units to the right

# --- Move the Cone to the Right ---
sim.setObjectPosition(cone_handle, -1, pos_right)
sim.step()  # Advance one simulation step
print("Position after moving right:", sim.getObjectPosition(cone_handle, -1))
time.sleep(1)  # Short delay for observation

# --- Move the Cone to the Left ---
sim.setObjectPosition(cone_handle, -1, pos_left)
sim.step()  # Advance one simulation step
print("Position after moving left:", sim.getObjectPosition(cone_handle, -1))
time.sleep(1)  # Short delay for observation

# --- Get and Print Initial Orientation ---
initial_orientation = sim.getObjectOrientation(cone_handle, -1)
print("Initial Cone Orientation:", initial_orientation)

# --- Rotate the Cone around the Z-axis ---
rotated_orientation = list(initial_orientation)
rotated_orientation[2] += math.pi / 4  # Rotate 45° (π/4 radians) around the Z-axis
sim.setObjectOrientation(cone_handle, -1, rotated_orientation)
sim.step()  # Advance one simulation step
print("Orientation after rotation:", sim.getObjectOrientation(cone_handle, -1))

# Stop the simulation
sim.stopSimulation()
