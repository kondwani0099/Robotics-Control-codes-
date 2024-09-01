# Type help("robodk.robolink") or help("robodk.robomath") for more information
# Press F5 to run the script
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/robodk.html
# Note: It is not required to keep a copy of this file, your Python script is saved with your RDK project


# Forward and backwards compatible use of the RoboDK API:
# Remove these 2 lines to follow python programming guidelines
from robodk import *      # RoboDK API
from robolink import *    # Robot toolbox
# Link to RoboDK
RDK = Robolink()

robot =  RDK.Item('KUKA KR 150 R3300 K prime')
home =  RDK.Item('Home')
target =  RDK.Item('Target 1')

poseref =  target.Pose()

#move the robot to hoome then to the center
robot.MoveJ(home)
robot.MoveJ(target)

#make an hexagon around the center
for i in range(10):
    ang = i*2*pi/6 # angle : 0 , 60 ,120 ,,,,
    posei = poseref*rotz(ang)*trans1(300,0,0)*rotz(-ang)
    robot.MoveL(posei)
#robot move back to the center ,then home :
    robot.MoveL(target)
    robot.MoveJ(home)

