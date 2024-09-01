import socket
from robodk.robolink import *  # RoboDK API
from robodk.robomath import *  # Robot toolbox

# Initialize RoboDK and the robot
RDK = Robolink()
robot = RDK.Item('KUKA KR')

def move_robot(x, y, z):
    target = transl(x, y, z)
    robot.MoveJ(target)

# Create a server to receive commands from MediaPipe script
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('localhost', 65432))
server_socket.listen(1)

print("Waiting for connection...")
while True:
    client_socket, addr = server_socket.accept()
    data = client_socket.recv(1024)
    if not data:
        break
    x, y, z = map(float, data.decode().split(','))
    print(f"Moving robot to: x={x}, y={y}, z={z}")
    move_robot(x, y, z)
    client_socket.close()
