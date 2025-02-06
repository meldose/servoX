# Vision controller side :


from neurapy.socket_interface.robot import Robot

import socket 

# Replace with the IP address and port of your robotic controller
ROBOT_IP = '192.168.1.2'
ROBOT_PORT = 65432
 
# Sample object coordinates
object_coordinates = {'x': 123.45, 'y': 67.89, 'z': 10.11}
 
# Create a TCP/IP socket
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((ROBOT_IP, ROBOT_PORT))
    # Send coordinates as a string (you can use JSON or another format)
    s.sendall(str(object_coordinates).encode('utf-8'))
    print("Coordinates sent!")


# Robotic controller side :
    
import socket
 
# Bind to the IP and port
HOST = '0.0.0.0'  # Listen on all available interfaces
PORT = 65432

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print("Waiting for connection...")
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        data = conn.recv(1024)
        if data:
            coordinates = eval(data.decode('utf-8'))
            print(f"Received coordinates: {coordinates}")
            # Here you’d include the logic to move the robot based on coordinates