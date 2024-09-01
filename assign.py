import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np

# Define DH parameters
# [theta d a alpha]
dh_params = [
    [0, 260.4, 0, -np.pi/2],
    [0, 0, 228.6, 0],
    [0, 0, 228.6, 0],
    [0, 95, 0, -np.pi/2],
    [0, 171.5, 0, 0]
]

# Create links
L = []
for param in dh_params:
    theta, d, a, alpha = param
    L.append(rtb.RevoluteDH(d=d, a=a, alpha=alpha))

# Create the robot model
rhino_xr3 = rtb.DHRobot(L, name='Rhino XR3')

# Define joint angles (in degrees, then converted to radians)
joint_angles = [0, 0, 0, 0, 0]
joint_angles = np.deg2rad(joint_angles)

# Compute forward kinematics
T = rhino_xr3.fkine(joint_angles)
print('Transformation matrix T:')
print(T)

# Plot the robot
rhino_xr3.plot(joint_angles, block=True)
