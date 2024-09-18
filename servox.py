from neurapy.robot import Robot
import time
import copy
import signal
import sys
import numpy as np
import quaternion

r = Robot()

target_1 = 0.3
target_2 = 0.2

#Switch to external servo mode
r.activate_servo_interface('position')
cart_pose_length = 7 # X,Y,Z,qw,qx,qy,qz
target = copy.deepcopy(r.get_current_cartesian_pose())
print(target)

target[1] -= target_1
velocity = [0.15]*7 
acceleration = [2.]*7
error_code = r.movelinear_online(target, velocity, acceleration)

time.sleep(2)

target = copy.deepcopy(r.get_current_cartesian_pose())
target[1] += target_2
error_code = r.movelinear_online(target, velocity, acceleration)

time.sleep(3)

r.stop_movelinear_online()
print("Robot stopped")
time.sleep(6)
r.deactivate_servo_interface()
r.stop()