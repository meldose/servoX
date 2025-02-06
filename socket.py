# Vision Controller Side

import socket
import json

ROBOT_IP = "192.168.1.13"  # Updated to match the robotic controller IP
ROBOT_PORT = 11003

# Sample object coordinates
object_coordinates = {'x': 123.45, 'y': 67.89, 'z': 10.11}

# Create a TCP/IP socket
try:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(5)  # Added timeout for the socket
        s.connect((ROBOT_IP, ROBOT_PORT))
        # Send coordinates as JSON
        s.sendall(json.dumps(object_coordinates).encode('utf-8'))
        print("Coordinates sent successfully!")
except socket.timeout:
    print("Connection timed out.")
except ConnectionRefusedError:
    print("Connection refused. Check if the robotic controller is running.")
except Exception as e:
    print(f"Failed to send coordinates: {e}")


# Robotic Controller Side

HOST = '192.168.1.13'
PORT = 11003

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow address reuse
    s.bind((HOST, PORT))
    s.listen()
    s.settimeout(10)  # Added timeout to prevent infinite blocking
    print("Waiting for connection...")

    while True:  # Allow continuous listening for connections
        try:
            conn, addr = s.accept()
            with conn:
                print(f"Connected by {addr}")
                data_buffer = b''
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break
                    data_buffer += data

                try:
                    # Use JSON for safe parsing
                    coordinates = json.loads(data_buffer.decode('utf-8'))
                    print(f"Received coordinates: {coordinates}")

                    # Graceful shutdown command
                    if coordinates.get('command') == 'shutdown':
                        print("Shutting down server...")
                        break
                except json.JSONDecodeError:
                    print("Received invalid JSON data.")
                except Exception as e:
                    print(f"Error handling data from {addr}: {e}")
        except socket.timeout:
            print("No incoming connections. Server is still listening...")
        except Exception as e:
            print(f"Error accepting connection: {e}")
