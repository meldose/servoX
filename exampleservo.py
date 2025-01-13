from neurapy.robot import Robot # importing robot module
import time # importing time module
import numpy as np # importing numpy
import signal # importing signal
import sys # importing sys

def signal_handler(signum, frame):
    print("Signal received, stopping robot")
    r.stop()  # This should be the method to stop all robot movements
    r.deactivate_servo_interface()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

r = Robot()
r.activate_servo_interface('position')

# Define detailed paths for each movement
# Example: List of positions the robot should move through linearly
path_1 = np.array([[1.5708, -1.5708, 0.7854, -0.7854, 1.5708, 0], [0, 1.5708, -1.5708, 0.7854, -0.7854, 1.5708], [1.0, -0.5, 0.5, 0, 1.0, 0.5]])  # More points added
path_2 = np.array([[1.2, -0.7, 0.7, 0, 1.2, 0.8], [1.0, -0.5, 0.5, 0, 1.0, 0]])

paths = [path_1, path_2]
for path in paths:
    print("Moving through path:", path)
    try:
        # Assuming move_linear can take multiple points as a list of lists
        error_code = r.move_linear(path.tolist(), velocity=0.15, acceleration=2.0)  # Adjust parameters as necessary
        if error_code != 0:
            print("Error encountered:", error_code)
            break
    except Exception as e:
        print("An exception occurred:", str(e))
        break
    time.sleep(10)

r.deactivate_servo_interface()
r.stop()
print("Robot stopped")

