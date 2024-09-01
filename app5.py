import pybullet as p
import pybullet_data
import time

# Connect to PyBullet in GUI mode
p.connect(p.GUI)

# Set the search path to find URDF files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a plane and the KUKA robot arm
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

# Define the initial and target positions for the cube
initial_cube_position = [0.5, 0, 0.05]
target_cube_position = [0.3, 0.3, 0.05]

# Load the cube at the initial position
cube_id = p.loadURDF("cube_small.urdf", initial_cube_position)

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
    p.setJointMotorControl2(robot_id, 7, p.POSITION_CONTROL, targetPosition=0.04)
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)

def close_gripper(robot_id):
    p.setJointMotorControl2(robot_id, 7, p.POSITION_CONTROL, targetPosition=0.0)
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)

def move_arm_to_target(robot_id, target_pos, target_orient):
    joint_positions = p.calculateInverseKinematics(robot_id, 6, target_pos, target_orient)
    move_to_position(robot_id, range(7), joint_positions)

# Move to the cube's initial position
initial_grasp_position = initial_cube_position[:]
initial_grasp_position[2] += 0.15  # Move above the cube initially
target_orientation = p.getQuaternionFromEuler([0, 0, 0])
move_arm_to_target(robot_id, initial_grasp_position, target_orientation)
time.sleep(1)

# Lower the arm to grasp the cube
initial_grasp_position[2] -= 0.14
move_arm_to_target(robot_id, initial_grasp_position, target_orientation)
time.sleep(1)

# Close the gripper to grasp the cube
close_gripper(robot_id)
time.sleep(1)

# Lift the cube
initial_grasp_position[2] += 0.15
move_arm_to_target(robot_id, initial_grasp_position, target_orientation)
time.sleep(1)

# Move to the target position
target_grasp_position = target_cube_position[:]
target_grasp_position[2] += 0.15  # Move above the target position
move_arm_to_target(robot_id, target_grasp_position, target_orientation)
time.sleep(1)

# Lower the arm to place the cube
target_grasp_position[2] -= 0.14
move_arm_to_target(robot_id, target_grasp_position, target_orientation)
time.sleep(1)

# Open the gripper to place the cube
open_gripper(robot_id)
time.sleep(1)

# Lift the arm
target_grasp_position[2] += 0.15
move_arm_to_target(robot_id, target_grasp_position, target_orientation)
time.sleep(1)

# Run an interactive simulation loop
while True:
    p.stepSimulation()
    time.sleep(1./240.)

# Disconnect from PyBullet
p.disconnect()
