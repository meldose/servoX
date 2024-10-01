from neurapy.robot import Robot
import time
import copy
import signal
import sys
import numpy as np
import quaternion

# def signal_handler(signum, frame):
#     print("Signal Handler called")
#     r.deactivate_servo_interface()
#     r.stop()
#     sys.exit(0)

r = Robot()

target_1 = 0.3
target_2 = 0.25

#Switch to external servo mode
r.activate_servo_interface('position')
cart_pose_length = 7 # X,Y,Z,qw,qx,qy,qz
target = copy.deepcopy(r.get_current_cartesian_pose())
print(target)

target[0] -= target_1
velocity = [0.15]*6
acceleration = [2.]*6
error_code = r.movelinear_online(target, velocity, acceleration)

time.sleep(10)
# r.stop_movelinear_online()


target = copy.deepcopy(r.get_current_cartesian_pose())
target[2] += target_2
error_code = r.movelinear_online(target, velocity, acceleration)

time.sleep(5)

# r.stop_movelinear_online()
print("Robot stopped")
time.sleep(2)
r.deactivate_servo_interface()
r.stop()
