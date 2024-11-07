from neurapy.robot import Robot # import robot module
import time # import time module
import copy # import copy moudule
import signal # import signal module
import sys # import sys module
import numpy as np # import numpy module 
import quaternion # impoort quaternion module

signal.signal(signal.SIGINT, signal_handler)
    # def signal_handler(signum, frame):

r = Robot() # assigning the robot module to r variable

#Switch to external servo mode
r.activate_servo_interface('position')

target_1 = [0.3, 0.25, 0.1] # added for testing the target position 
target_2 = [0.25, 0.3, 0.2] # added second target position for testing 

for target in [target_1, target_2]: # checking the values are there in target 1 and 2
    print("Target:", target) # printing the target position
    current_pose = copy.deepcopy(r.get_current_cartesian_pose()) # adding the current position to an variable
    current_pose[:3] += target # adding the current position of the robot to the target position
    velocity = [0.15]*6 # velocity is set to 0.15   
    acceleration = [2.]*6 # acceleration is set to 2
    error_code = r.movelinear_online(current_pose, velocity, acceleration)
    time.sleep(10) # setting the time  for sleep as 10 seconds
    target[0] -= target_1 # increasing the target position by 1


r.stop_movelinear_online() # stop the robot movement    
print("Robot stopped") # robot stopped 
time.sleep(2) # sleep for 2 seconds
r.deactivate_servo_interface() # deactivating the servo interface
r.stop() # stopped the robot

