import cv2
import mediapipe as mp
import socket

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()
mp_drawing = mp.solutions.drawing_utils

# Define a function to send commands to the RoboDK server
def send_command(x, y, z):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('localhost', 65432))
    client_socket.send(f"{x},{y},{z}".encode())
    client_socket.close()

# Start capturing video
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            # Example: Use thumb tip (landmark 4) coordinates to control the robot
            thumb_tip = hand_landmarks.landmark[4]
            x = int(thumb_tip.x * frame.shape[1])
            y = int(thumb_tip.y * frame.shape[0])
            z = int(thumb_tip.z * 1000)  # Scale z for demonstration
            
            # Send command to the robot
            send_command(x, y, z)

    cv2.imshow('Hand Tracking', frame)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
