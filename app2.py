import pybullet as p
import pybullet_data
import time
import numpy as np

# Connect to PyBullet in GUI mode
p.connect(p.GUI)

# Set the search path to find URDF files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a plane and a robot (Franka Panda in this case)
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# Load an object to manipulate (e.g., a cube)
cube_id = p.loadURDF("cube_small.urdf", [0.5, 0, 0.5])

# Set gravity
p.setGravity(0, 0, -9.8)

# Set initial camera parameters
cam_target_pos = [0.5, 0, 0.5]
cam_distance = 1.5
cam_yaw = 45
cam_pitch = -30

# Update the camera view
p.resetDebugVisualizerCamera(cameraDistance=cam_distance,
                             cameraYaw=cam_yaw,
                             cameraPitch=cam_pitch,
                             cameraTargetPosition=cam_target_pos)

def move_to_position(robot_id, joint_indices, positions):
    for i, joint_index in enumerate(joint_indices):
        p.setJointMotorControl2(robot_id, joint_index, p.POSITION_CONTROL, targetPosition=positions[i])
    p.stepSimulation()

def open_gripper(robot_id):
    p.setJointMotorControl2(robot_id, 9, p.POSITION_CONTROL, targetPosition=0.04)
    p.setJointMotorControl2(robot_id, 10, p.POSITION_CONTROL, targetPosition=0.04)
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)

def close_gripper(robot_id):
    p.setJointMotorControl2(robot_id, 9, p.POSITION_CONTROL, targetPosition=0.0)
    p.setJointMotorControl2(robot_id, 10, p.POSITION_CONTROL, targetPosition=0.0)
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)

def move_arm_to_target(robot_id, target_pos, target_orient):
    joint_positions = p.calculateInverseKinematics(robot_id, 11, target_pos, target_orient)
    move_to_position(robot_id, range(7), joint_positions)

def get_end_effector_state(robot_id):
    # Get the state of the end-effector (link index 11 for Franka Panda)
    state = p.getLinkState(robot_id, 11)
    pos = state[4]  # Link world position
    orient = state[5]  # Link world orientation
    return pos, orient

# Move to the object
target_position = [0.5, 0, 0.4]
target_orientation = p.getQuaternionFromEuler([0, 0, 0])
move_arm_to_target(robot_id, target_position, target_orientation)
time.sleep(1)

# Close the gripper to grasp the object
close_gripper(robot_id)
time.sleep(1)

# Get the current state of the end-effector
end_effector_pos, end_effector_orient = get_end_effector_state(robot_id)
print("End-effector position:", end_effector_pos)
print("End-effector orientation:", end_effector_orient)

# Move to a new location
new_position = [0.3, 0, 0.4]
move_arm_to_target(robot_id, new_position, target_orientation)
time.sleep(1)

# Open the gripper to place the object
open_gripper(robot_id)
time.sleep(1)

# Run the simulation
for _ in range(1000):
    p.stepSimulation()
    time.sleep(1./240.)

# Disconnect from PyBullet
p.disconnect()
