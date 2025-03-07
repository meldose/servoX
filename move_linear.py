import copy # importing copy module

class ServoX: # defining servoX

    def __init__(self,robot):
        self.robot = robot # setting the robot

    def movelinear_online(self,message,*args,**kwargs):# defining movelinear_online function

      # message = [x/1000 for x in message] # converting the values to mm
        z_offset = 0.04
        x = message[0] / 1000 # Scale values
        y = message[1] / 1000 # Scale values
        z = message[2] / 1000 + z_offset  # created an offset
        w = message[3] # orientation values
        ex= message[4] # orientation values
        ey= message[5] # orientation values
        ez= message[6] # orientation values
        

        new_message = [x, y,z,w,ex,ey,ez] # added new order for quaternion values
        
        print(message) # printing the message
        print(new_message) # printing the new ordered message

        r = self.robot # setting the robot
        
        #Switch to external servo mode
        r.activate_servo_interface('position') # activating the servo interface
   
        cart_pose_length = 7 # X,Y,Z,qw,qx,qy,qz
        velocity = [0.6]*6 # setting the velocity 
        acceleration = [8.0]*6 # setting the acceleration
        target = copy.deepcopy(r.get_current_cartesian_pose()) # getting the current cartesian poses
        time.sleep(0.9) # setting time sleep

        target=new_message # setting the target position
        target[2] += 0.01
        # target = [new_message[0], new_message[1], new_message[2], target[3], target[4], target[5], target[6]]
        error_code = r.movelinear_online(target, velocity, acceleration) # moving the robot
        time.sleep(0.9) # setting time sleep

        r.deactivate_servo_interface() # deactivating the servo interface
        r.gripper("off") # setting gripper close position
        # r.move_joint("P50") # moving to P34
        # r.gripper("off") # setting gripper close position
        r.move_joint("P54") # moving to P33
        r.gripper("on") # setting gripper on
        r.move_joint("P52") # moving to P32
        # r.stop() # stopping the robot
    

r.set_mode("Automatic") # setting the mode to automatic
r.gripper("on") # setting the gripper on
r.move_joint("P52") # moving to P32