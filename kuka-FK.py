from robodk import robolink, robomath
from robolink import *  # API to communicate with RoboDK
from robodk import *    # basic matrix operations

RL = robolink.Robolink()

def FK_Robot(dh_table, joints):
    """Computes the forward kinematics of the robot.
    dh_table must be in mm and radians, the joints vector must be in degrees."""
    Habs = []
    Hrel = []
    nlinks = len(dh_table)
    HiAbs = eye(4)
    for i in range(nlinks):
        [rz, tx, tz, rx] = dh_table[i]
        rz = rz + joints[i] * pi / 180
        Hi = dh(rz, tx, tz, rx)
        HiAbs = HiAbs * Hi
        Hrel.append(Hi)
        Habs.append(HiAbs)
    return [HiAbs, Habs, Hrel]

def Frames_setup_absolute(frameparent, nframes):
    """Adds nframes to frameparent"""
    frames = []
    for i in range(nframes):
        newframe = frameparent.RL().AddFrame('frame %i' % (i + 1), frameparent)
        newframe.setPose(transl(0, 0, 100 * i))
        frames.append(newframe)
    return frames

def Frames_setup_relative(frameparent, nframes):
    """Adds nframes cascaded to frameparent"""
    frames = []
    parent = frameparent
    for i in range(nframes):
        newframe = frameparent.RL().AddFrame('frame %i' % (i + 1), parent)
        parent = newframe
        newframe.setPose(transl(0, 0, 100))
        frames.append(newframe)
    return frames

def Set_Items_Pose(itemlist, poselist):
    """Sets the pose (3D position) of each item in itemlist"""
    for item, pose in zip(itemlist, poselist):
        item.setPose(pose)

def are_equal(j1, j2):
    """Returns True if j1 and j2 are equal, False otherwise"""
    if j1 is None or j2 is None:
        return False
    sum_diffs_abs = sum(abs(a - b) for a, b in zip(j1, j2))
    if sum_diffs_abs > 1e-3:
        return False
    return True

# DH table of the robot: KUKA KR 150 R3300 K Prime
DH_Table = [
    [0, 150, 750, 0],      # Link 1: alpha=0, a=150, d=750, theta=0
    [-90*pi/180, 1200, 0, 0],  # Link 2: alpha=-90, a=1200, d=0, theta=0
    [0, 215, 0, 0],        # Link 3: alpha=0, a=215, d=0, theta=0
    [-90*pi/180, 220, 1350, 0],  # Link 4: alpha=-90, a=220, d=1350, theta=0
    [90*pi/180, 0, 0, 0],  # Link 5: alpha=90, a=0, d=0, theta=0
    [-90*pi/180, 0, 330, 0]  # Link 6: alpha=-90, a=0, d=330, theta=0
]

# Degrees of freedom: (6 for KUKA KR 150 R3300 K Prime)
DOFs = len(DH_Table)

# Get the robot:
robot = RL.Item('KUKA KR 150 R3300 K Prime')

# Cleanup of all items containing "Robot base"
while True:
    todelete = RL.Item('Robot base')
    # Make sure an item was found
    if not todelete.Valid():
        break
    # Delete only frames
    if todelete.Type() == ITEM_TYPE_FRAME:
        print('Deleting: ' + todelete.Name())
        todelete.Delete()

# Setup the parent frames for the test:
parent_frameabs = RL.AddFrame('Robot base (absolute frames)')
parent_framerel = RL.AddFrame('Robot base (relative frames)')

# Setup the child frames for the test:
frames_abs = Frames_setup_absolute(parent_frameabs, DOFs)
frames_rel = Frames_setup_relative(parent_framerel, DOFs)

last_joints = None

tic()
while True:
    # Get the current robot joints
    joints = tr(robot.Joints())
    joints = joints.rows[0]

    # Do not repaint if joints are the same
    if are_equal(joints, last_joints):
        continue

    # If joints changed, compute the forward kinematics for this position
    [Hrobot, HabsList, HrelList] = FK_Robot(DH_Table, joints)

    # Turn off rendering after every Item call while we update all frames:
    RL.Render(False)
    # Update all frames
    Set_Items_Pose(frames_abs, HabsList)
    Set_Items_Pose(frames_rel, HrelList)
    # Render and turn on rendering
    RL.Render(True)

    last_joints = joints

    # Display some information:
    toc()
    print('Current robot joints:')
    print(joints)
    print('Pose of the robot (forward kinematics):')
    print(Hrobot)
    print('\n\n')
