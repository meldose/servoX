from neurapy.robot import Robot # importing robot module
import time # importing time module
from ruckig import InputParameter, OutputParameter, Result, Ruckig # importing ruckig module
 
r = Robot() #settig r as the variable for the Robot
r.gripper("on") # setting gripper on

def servo_j(): # defining function for servoJ
    # #Switch to external servo mode

    # message = [x/1000 for x in message] # converting the values to mm
    
    # x = message[0]  # setting the values
    # y = message[1]  # setting the values
    # z = message[2]  # setting the values
    # a = message[3]  # setting the values
    # b = message[4]  # setting the values
    # c = message[5]  # setting the values
    # d = message[6]  # setting the values
    
    # print(message) # printing the message
    
    # new_message = [x,y,z,d,a,b,c] # added new order for quaternion values
    # print(new_message) # printing the new ordered message

    r.activate_servo_interface('position') # activating the servo interface
    dof = 6 # setting the DOF as 6 
    otg = Ruckig(dof, 0.001)  # DoFs, control cycle

    inp = InputParameter(dof) # setting the input parameter
    out = OutputParameter(dof) # setting the output parameter
 
    inp.current_position = r.get_current_joint_angles() # getting the current joint angles
    inp.current_velocity = [0.]*dof
    inp.current_acceleration = [0.]*dof
 
    inp.target_position = [0.561036550354565, -0.8075385726899302, -0.8665202488853906, -0.07588653308042255, -1.5022882144652068, 2.0441351475042864] # target positon
    #inp.target_position = [x,y,z,a,b,c,d]
    inp.max_velocity = [1.0]*dof # setting up the maximum velocity 
    inp.max_acceleration = [5]*dof # setting up the maximum acceleration
   
    inp.target_acceleration = [0.]*dof # setting the target acceleration as zero.
    r.gripper("on") # setting the gripper in On position.
    inp.max_velocity = [1.0]*dof
    inp.max_acceleration = [5]*dof
    inp.max_jerk = [1.0]*dof
    res = Result.Working

    res = Result.Working
 
    while res == Result.Working:
        error_code = 0

        res = otg.update(inp, out)

        position = out.new_position # setting the new position 
        velocity = out.new_velocity # setting the new velocity
        acceleration = out.new_acceleration # setting the new acceleration 
 
        error_code = r.servo_j(position, velocity, acceleration) # passing the error code variable with having servo_j function having position, velocity and acceleration.
        # print(error_code) # checking if the error is there or not 
        scaling_factor = r.get_servo_trajectory_scaling_factor() # getting the servo trajectory scaling factors.
        out.pass_to_input(inp)
        time.sleep(0.001) # setting the time sleep to 0.001 seconds

    r.deactivate_servo_interface() # deactivating the servo interface
 
    # r.stop() # stopped the robot

servo_j() # calling the servo_j function
r.gripper("off") # setting gripper off

