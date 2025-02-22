from neurapy.robot import Robot # impoting modules
import time # importing time module
from ruckig import InputParameter, OutputParameter, Result, Ruckig # importing module like Ouputparameter ,inputparameter, result and ruckig
import copy # importing copy module

r=Robot() # deifning robot 
r.gripper("on") # seting gripper on condition

def servo_x(message): # function for servo x creating

    
    r = Robot()

    message = [x/1000 for x in message] # converting the values to mm
    
    x = message[0]  # setting the values
    y = message[1]  # setting the values
    z = message[2]  # setting the values
    a = message[3]  # setting the values
    b = message[4]  # setting the values
    c = message[5]  # setting the values
    d = message[6]  # setting the values
    
    print(message) # printing the message
    
    new_message = [x,y,z,d,a,b,c] # added new order for quaternion values
    print(new_message) # printing the new ordered message


    #Switch to external servo mode
    r.activate_servo_interface('position') #  activating the servo interface

    cart_pose_length = 7 #X,Y,Z,qw,qx,qy,qz 

    otg = Ruckig(cart_pose_length, 0.001)  # control cycle
    inp = InputParameter(cart_pose_length) # setting the inputparameter with cart pose length
    out = OutputParameter(cart_pose_length) # setting the outputparmeter with cart pose length

    inp.current_position = r.get_current_cartesian_pose() # getting the current pose of the robot
    target = copy.deepcopy(inp.current_position) # setting the target by copying the current position
    inp.target_position = [new_message[0], new_message[1], new_message[2], target[3], target[4], target[5], target[6]] # setting the target_position
    # inp.target_position = [-0.529,-0.380,0.058,0.590,-0.587,0.366,-0.418] # providing the target position
    inp.current_velocity = [0.]*cart_pose_length # mutliplying the initila velocity with cart pose lenght 
    inp.current_acceleration = [0.]*cart_pose_length # mutliplying the current acceleration with cart pose length

    target = copy.deepcopy(inp.current_position) # copying the current position of the robot 
    # target[0] += 0.2 # Move 200mm in X direction
    inp.target_position = [new_message[0], new_message[1], new_message[2], target[3], target[4], target[5], target[6]] # setting the target_position
    # inp.target_position = [-0.529,-0.380,0.058,0.590,-0.587,0.366,-0.418] # initating the target position 
    inp.target_velocity = [0.]*cart_pose_length # defning the target velocity
    inp.target_acceleration = [0.]*cart_pose_length # definng the target acceleration

    inp.max_velocity = [0.5]*cart_pose_length # setting the maximum velocity with 0.5 times the cart pose lenght 
    inp.max_acceleration = [3]*cart_pose_length #se tting the max acceleration with 3 times the cart pose length
    inp.max_jerk = [10.]*cart_pose_length # setting the jerk values

    servox_proportional_gain = 25 # setting the servox propotional gain as 25

    velocity = [0.] * 6 #Since ruckig does not provide rotational velocity if quaternion is input, we can send 0 rotational feedforward velocity
    acceleration = [0.] * 6 #Since ruckig does not provide rotational acceleration if quaternion is input, we can send 0 rotational feedforward acceleration
    
    res= Result.Working # setting the condition for Result.Working

    while res == Result.Working: # checking the res is equal to Result.Working
        error_code = 0 # setting the error code as zero

        res = otg.update(inp, out) # setting the control cycle with input and output

        position = out.new_position # setting the position with the new postion

        for i in range(0,3): # Updating target translation velocity and accelerations
            velocity[i] = out.new_velocity[i] # setting the new_velocity
            acceleration[i] = out.new_acceleration[i] # setting the new acceleration
        
        error_code = r.servo_x(position, velocity, acceleration, servox_proportional_gain) # assigining the servox function with veolicty ,accelerationa and servoX_propotional_gain.
        print(error_code) # checking if there is an error or not 
        scaling_factor = r.get_servo_trajectory_scaling_factor() # getting the scalor factor
        out.pass_to_input(inp) # passing the out to the input
        time.sleep(0.001) # setting time 

    r.deactivate_servo_interface() # deactivating the servo interface

    r.stop() # stopping the robot

servo_x() # providing the target position for the robot
r.gripper("off") # setting gripper off condition