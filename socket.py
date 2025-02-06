# Vision controller side :

import socket
import json

import sys

from _typeshed import ReadableBuffer, Unused, WriteableBuffer
from collections.abc import Iterable
from enum import IntEnum, IntFlag
from io import BufferedReader, BufferedRWPair, BufferedWriter, IOBase, RawIOBase, TextIOWrapper
from typing import Any, Literal, Protocol, SupportsIndex
from typing_extensions import Self


ROBOT_IP = "192.168.1.5"
ROBOT_PORT = 11003

# Sample object coordinates
object_coordinates = {'x': 123.45, 'y': 67.89, 'z': 10.11}

# Create a TCP/IP socket
try:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((ROBOT_IP, ROBOT_PORT))
        # Send coordinates as JSON
        s.sendall(json.dumps(object_coordinates).encode('utf-8'))
        print("Coordinates sent successfully!")
except Exception as e:
    print(f"Failed to send coordinates: {e}")



# Robotic controller side :

HOST = '192.168.1.13'
PORT = 11003

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print("Waiting for connection...")

    while True:  # Allow continuous listening for connections
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            try:
                data = conn.recv(1024)
                if data:
                    # Use JSON for safe parsing
                    coordinates = json.loads(data.decode('utf-8'))
                    print(f"Received coordinates: {coordinates}")
                    # Add logic to move the robot based on coordinates
            except json.JSONDecodeError:
                print("Received invalid JSON data.")
            except Exception as e:
                print(f"Error handling connection: {e}")
