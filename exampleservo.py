from neurapy.robot import Robot
import time
import copy
import signal
import sys
import numpy as np
import quaternion

def signal_handler(signum, frame):
    print("Signal Handler called")
    r.deactivate_servo_interface()
    r.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
    # def signal_handler(signum, frame):

r = Robot()

#Switch to external servo mode
r.activate_servo_interface('position')

target_1 = [0.3, 0.25, 0.1]
target_2 = [0.25, 0.3, 0.2]

for target in [target_1, target_2]:
    print("Target:", target)
    current_pose = copy.deepcopy(r.get_current_cartesian_pose())
    current_pose[:3] += target
    velocity = [0.15]*6
    acceleration = [2.]*6
    error_code = r.movelinear_online(current_pose, velocity, acceleration)
    time.sleep(10)
    target[0] -= target_1

print("Robot stopped")
time.sleep(2)
r.deactivate_servo_interface()
r.stop()

