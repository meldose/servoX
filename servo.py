from neurapy.robot import Robot # importing robot module
import time  # importing the time module
from ruckig import InputParameter, Outputparameter, Result, Ruckig # importing the moduel inputparmater , Ouputparameter ,result and ruckig

r=Robot() # defining the robot 

r.activate_servo_interface("position") # method of activate servo interface

dof= 6 # defining the Degree of freedom

otg= Ruckig(dof, 0.001)
inp = InputParameter(dof) # defining the input parameter
out= Outputparameter(dof) # defining the output the parameter 

inp.current.position= r.get_current_joint_angles() # getting the current joint angles 
inp.current_velocity= [0]*dof # setting the current velocity to zero 
inp.current_acceleration = [0]*dof # setting the current acceleration to zero

inp.target_position = [0,0,0,0,0,0] # defining the target position
inp.target_velocity= [0]*dof # setting the target velocity as zero
inp.target_acceleration= [0]*dof # setting the target acceleration as zero

inp.max_velocity= [0.5]*dof #  setting the max velocity 
inp.max_acceleration = [3]*dof # setting the max acceleration
inp.max_jerk = [10]*dof
res = Result.Working # Assigning the result variable

while res==Result.Working: # while condition is true 
    error=0
    if error < 3: 
        res = otg.update(inp, out) # updating the Ruckig with new input and output parameters
        
        position= out.new_position # setting the new position
        velocity = out.new_velocity # setting the new velocity
        acceleration= out.new_acceleration #setting the new acceleration
        
        error_code =r.servo_j(position,velocity,acceleration) # assigning the servo_j method to error_code variable
        scaling_factor= r.get_servo_trajectory_scaling_factor() # getting the servo_trajectory scaling factor
        out.pass_to_input(inp) 
        time.sleep(1) # setting the time sleep for 1 seconds
    else:
        print("Servo in error, error code", error_code) # if no the servo is in error
        break
    
r.deactivate_servo_interface() # deactivating the servo interface

r.stop() # stopped the robot
