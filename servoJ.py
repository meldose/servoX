from neurapy.robot import Robot # importing robot module
import time # importing time module
from ruckig import InputParameter, OutputParameter, Result, Ruckig # importing ruckig module
 
r = Robot() #settig r as the variable for the Robot
r.gripper("on") # setting gripper on

def servo_j(): # defining function for servoJ
    #Switch to external servo mode
    r.activate_servo_interface('position') # activating the servo interface
    dof = 6 # setting the DOF as 6 
    otg = Ruckig(dof, 0.001)  # DoFs, control cycle

    inp = InputParameter(dof) # setting the input parameter
    out = OutputParameter(dof) # setting the output parameter
 
    inp.current_position = r.get_current_joint_angles() # getting the current joint angles
    inp.current_velocity = [0.]*dof
    inp.current_acceleration = [0.]*dof
 
    inp.target_position = [0.7085659080146489, -0.3768058027298319, -1.1482012029500146, 3.01013635480822, 1.5833808519529902, -2.279724948141282]
    r.gripper("on")
 
    inp.max_velocity = [0.5]*dof # setting up the maximum velocity 
    inp.max_acceleration = [3]*dof # setting up the maximum acceleration

    inp = InputParameter(dof) # setting the input parameter
    out = OutputParameter(dof) # setting the ouput parameters
    inp.current_position = r.get_current_joint_angles() # getting the current joint angles
    inp.current_velocity = [0.]*dof # setting the current velocity as zero
    inp.current_acceleration = [0.]*dof # setting the current acceleration as zero
 
    inp.target_position =  [0.7085659080146489, -0.3768058027298319, -1.1482012029500146, 3.01013635480822, 1.5833808519529902, -2.279724948141282] # providing the target position
    inp.target_acceleration = [0.]*dof # setting the target acceleration as zero.
    r.gripper("on") # setting the gripper in On position.
 
    inp.max_velocity = [0.5]*dof # defining the maximum velocity
    inp.max_acceleration = [3]*dof # defining the maximum acceleration

    inp.max_jerk = [10.]*dof
    res = Result.Working
 
    while res == Result.Working:
        error_code = 0

        res = otg.update(inp, out)

        position = out.new_position # setting the new position 
        velocity = out.new_velocity # setting the new velocity
        acceleration = out.new_acceleration # setting the new acceleration 
 
        error_code = r.servo_j(position, velocity, acceleration) # passing the error code variable with having servo_j function having position, velocity and acceleration.
        print(error_code) # checking if the error is there or not 
        scaling_factor = r.get_servo_trajectory_scaling_factor() # getting the servo trajectory scaling factors.
        out.pass_to_input(inp)
        time.sleep(0.001) # setting the time sleep to 0.001 seconds

    r.deactivate_servo_interface() # deactivating the servo interface
 
    r.stop() # stopped the robot

servo_j() # calling the servo_j function
r.gripper("off") # setting gripper off

