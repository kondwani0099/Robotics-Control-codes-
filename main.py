import numpy as np
import pybullet as p
import pybullet_data
import time

# Function to create a DH transformation matrix
def dh_transform(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Function to calculate forward kinematics
def forward_kinematics(dh_params):
    T = np.eye(4)
    for params in dh_params:
        T = np.dot(T, dh_transform(*params))
    return T

# DH parameters: [a, alpha, d, theta]
dh_params = [
    [0, np.pi/2, 0.3, 0],   # Link 1
    [0.3, 0, 0, 0],         # Link 2
    [0.3, 0, 0, 0],         # Link 3
    [0, np.pi/2, 0, 0],     # Link 4
    [0, -np.pi/2, 0.3, 0],  # Link 5
    [0, 0, 0.2, 0]          # Link 6
]

# Connect to PyBullet in GUI mode
p.connect(p.GUI)

# Set the search path to find URDF files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a plane and a simple robotic arm (UR5 as an example)
# plane_id = p.loadURDF("plane.urdf")
# robot_id = p.loadURDF("ur5/ur5.urdf", useFixedBase=True)
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# Set gravity
p.setGravity(0, 0, -9.8)

# Function to move the robotic arm
def move_robot(dh_params):
    joint_angles = [params[3] for params in dh_params]
    for i in range(len(joint_angles)):
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=joint_angles[i])
    for _ in range(240):
        p.stepSimulation()
        time.sleep(1./240.)

# Define different movement modes
def mode1():
    # Example movement: Move to a specific position
    dh_params[0][3] = np.pi/4
    dh_params[1][3] = -np.pi/4
    dh_params[2][3] = np.pi/4
    move_robot(dh_params)

def mode2():
    # Example movement: Move to another specific position
    dh_params[0][3] = -np.pi/4
    dh_params[1][3] = np.pi/4
    dh_params[2][3] = -np.pi/4
    move_robot(dh_params)

def mode3():
    # Example movement: Wave motion
    for _ in range(3):
        dh_params[0][3] = np.pi/4
        move_robot(dh_params)
        dh_params[0][3] = -np.pi/4
        move_robot(dh_params)

# Run different modes
mode1()
time.sleep(2)
mode2()
time.sleep(2)
mode3()

# Keep the simulation running
while True:
    p.stepSimulation()
    time.sleep(1./240.)

# Disconnect from PyBullet
p.disconnect()
