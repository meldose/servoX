from neurapy.robot import Robot # importing the Robot module
import time # importing the time module
import copy # importing the copy module

r = Robot() # setitng the robot

r.gripper("off") #setting the gripper off condition

def movelinear_online(self,*args,**kwargs): # defining the movelinear_online function
    target_1 = 0.3 # setting the target_1 as 0.3
    target_2 = 0.25 # settingthe target_2 as 0.25
 
    #Switch to external servo mode
    r.activate_servo_interface('position') # activating the servo interface
    cart_pose_length = 7 # X,Y,Z,qw,qx,qy,qz
    target = copy.deepcopy(r.get_current_cartesian_pose()) # getting the current cartesian poses
    print(target) # printing the target values

    # Move target_1 unit in -X direction
    target[0] -= target_1
    velocity = [0.15]*6 # setting the velocity 
    acceleration = [2.]*6 # setting the acceleration
    error_code = r.movelinear_online(target, velocity, acceleration)

    #Sleep for 5 sec to complete the motion.
    time.sleep(5)

    target = copy.deepcopy(r.get_current_cartesian_pose())
    #Move target_2 units in +Z direction
    target[2] += target_2
    error_code = r.movelinear_online(target, velocity, acceleration)

    #Sleep for 5 sec to complete the motion
    time.sleep(5)

    print("Robot stopped") #  print the statment as Robot stopped
    r.deactivate_servo_interface() # deactivating the servo interface 
    r.stop() # stopping the robot 
    
movelinear_online() # calling the fucntion 
r.gripper("on") # setting the gripper in on position 