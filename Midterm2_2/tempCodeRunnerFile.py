from coppeliasim_zmqremoteapi_client import RemoteAPIClient

print("🔌 Connecting...")
client = RemoteAPIClient()
sim = client.getObject('sim')  # ✅ getObject, yanlış olan get_object değil

print("✅ Connected.")
print("🧠 Simulation state:", sim.getSimulationState())
