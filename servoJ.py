from neurapy.robot import Robot # importing robot module
import time # importing time module
from ruckig import InputParameter, OutputParameter, Result, Ruckig # importing ruckig module
 
r = Robot() #settig r as the variable for the Robot
r.gripper("on") # setting gripper on

def servo_j(): # defining function for servoJ
    #Switch to external servo mode
    r.activate_servo_interface('position')
    dof = 6
    otg = Ruckig(dof, 0.001)  # DoFs, control cycle
    inp = InputParameter(dof)
    out = OutputParameter(dof)
    inp.current_position = r.get_current_joint_angles()
    inp.current_velocity = [0.]*dof
    inp.current_acceleration = [0.]*dof
 
    inp.target_position = [0.6152615661377963, -0.4838400657555952, -1.1460763967934788, 3.0131120004489866, 1.4672724852976176, -2.358734209070975] # providing the target position
    inp.target_acceleration = [0.]*dof
    r.gripper("on")
 
    inp.max_velocity = [0.5]*dof # defining the maximum velocity
    inp.max_acceleration = [3]*dof # defining the maximum acceleration
    inp.max_jerk = [10.]*dof
    res = Result.Working
 
    while res == Result.Working:
        error_code = 0

        res = otg.update(inp, out)

        position = out.new_position
        velocity = out.new_velocity
        acceleration = out.new_acceleration
 
        error_code = r.servo_j(position, velocity, acceleration)
        print(error_code) # checking if the error is there or not 
        scaling_factor = r.get_servo_trajectory_scaling_factor()
        out.pass_to_input(inp)
        time.sleep(0.001)

    r.deactivate_servo_interface() # deactivating the servo interface
 
    r.stop() # stopped the robot

servo_j() # calling the servo_j function 
r.gripper("off") # setting gripper off

