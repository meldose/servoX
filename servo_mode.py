from neurapy.robot import Robot # importing the robot module
import time # importing time module
from ruckig import InputParameter, OutputParameter, Result, Ruckig
import copy # importing copy module

r = Robot() # setting the robot 
r.set_override(0.5) #setting the override to 50%
r.set_mode("Automatic") # setting the mode to Automatic mode
r.gripper("on") # setting the gripper to ON position
def movelinear_online(): # defining the function movelinear_online 
    #Switch to external servo mode
    r.activate_servo_interface('position')

    cart_pose_length = 7  # X, Y, Z, qw, qx, qy, qz

    otg = Ruckig(cart_pose_length, 0.001)  # control cycle
    inp = InputParameter(cart_pose_length) # defining the input parameter
    out = OutputParameter(cart_pose_length) # definig the ouput parameter

    # Set current position (your provided coordinates)
    inp.current_position = [-0.515,-0.378,0.390,0.151,0.888,-0.168,0.401] # giving the target position
    inp.current_velocity = [0.] * cart_pose_length # setting the current velocity to zero
    inp.current_acceleration = [0.] * cart_pose_length # setting the current acceleration to zero

    # Target position (displace 200mm in X direction)
    target = copy.deepcopy(inp.current_position)
    target[0] += 0.2  # Move 200mm in the X direction (add to X component)
    inp.target_position = target
    inp.target_velocity = [0.] * cart_pose_length # setting the target_velocity
    inp.target_acceleration = [0.] * cart_pose_length # setting the target_acceleration to zero

    # Motion limits
    inp.max_velocity = [0.5] * cart_pose_length # setting the max_velocity
    inp.max_acceleration = [3] * cart_pose_length # setting the max acceleration
    inp.max_jerk = [10.] * cart_pose_length 
    res = Result.Working

    servox_proportional_gain = 25 # setting the servox_propotional_gain as 25

    while res == Result.Working:

        error_code = 0
        if(error_code < 3):
            res = otg.update(inp, out)

            position = out.new_position # setting new position
            velocity = out.new_velocity  # setting new velocity
            acceleration = out.new_acceleration # setting new acceleration

            error_code = r.servo_x(position, velocity, acceleration, servox_proportional_gain)
            scaling_factor = r.get_servo_trajectory_scaling_factor()
            out.pass_to_input(inp)
            time.sleep(0.001) #setting the time sleep as 0.001 seconds
        else:
            print("Servo in error, error code, ", error_code)
            break
    r.deactivate_servo_interface() # deactivate the servo interface

    r.stop() # stop the module 

movelinear_online() # calling the function
r.gripper("off") # setting the gripper to off condition

