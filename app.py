import pybullet as p
import pybullet_data
import time

# Connect to PyBullet
p.connect(p.GUI)

# Set the search path to find URDF files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a plane and a robot
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# Set gravity
p.setGravity(0, 0, -9.8)

# Run simulation and visualize
for _ in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

# Disconnect from PyBullet
p.disconnect()
