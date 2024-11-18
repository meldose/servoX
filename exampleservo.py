from neurapy.robot import Robot
import time
import numpy as np
import signal
import sys

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
path_1 = np.array([[0.3, 0.25, 0.1], [0.35, 0.25, 0.15], [0.3, 0.30, 0.1]])  # More points added
path_2 = np.array([[0.25, 0.3, 0.2], [0.2, 0.35, 0.25], [0.25, 0.35, 0.2]])

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
