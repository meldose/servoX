from neurapy.robot import Robot # impoting modules
import time # importing time module
from ruckig import InputParameter, OutputParameter, Result, Ruckig # importing module like Ouputparameter ,inputparameter, result and ruckig
import copy # importing copy module

r=Robot() # deifning robot 
r.gripper("on") # seting gripper on condition

def servo_x(self,*args,**kwargs): # function for servo x creating 
    
    r = Robot()

    #Switch to external servo mode
    r.activate_servo_interface('position')

    cart_pose_length = 7 #X,Y,Z,qw,qx,qy,qz 

    otg = Ruckig(cart_pose_length, 0.001)  # control cycle
    inp = InputParameter(cart_pose_length) # setting the inputparameter with cart pose length
    out = OutputParameter(cart_pose_length) # setting the outputparmeter with cart pose length

    inp.current_position = r.get_current_cartesian_pose()
    inp.target_position = [-0.522, -0.315, 0.120,-3.02,-0.06,1.41] # getting the cartesian pose
    inp.current_velocity = [0.]*cart_pose_length # mutliplying the initila velocity with cart pose lenght 
    inp.current_acceleration = [0.]*cart_pose_length # mutliplying the current acceleration with cart pose length

    target = copy.deepcopy(inp.current_position) # copying the current position of the robot 
    target[0] += 0.2 # Move 200mm in X direction
    inp.target_position = target # initating the target position 
    inp.target_velocity = [0.]*cart_pose_length # defning the target velocity
    inp.target_acceleration = [0.]*cart_pose_length # definng the target acceleration

    inp.max_velocity = [0.5]*cart_pose_length # setting the maximum velocity with 0.5 times the cart pose lenght 
    inp.max_acceleration = [3]*cart_pose_length #se tting the max acceleration with 3 times the cart pose length
    inp.max_jerk = [10.]*cart_pose_length # setting the jerk values

    servox_proportional_gain = 25 # setting the servox propotional gain as 25

    velocity = [0.] * 6 #Since ruckig does not provide rotational velocity if quaternion is input, we can send 0 rotational feedforward velocity
    acceleration = [0.] * 6 #Since ruckig does not provide rotational acceleration if quaternion is input, we can send 0 rotational feedforward acceleration

    while res == Result.Working:
        error_code = 0

        res = otg.update(inp, out)

        position = out.new_position

        for i in range(0,3): # Updating target translation velocity and accelerations
            velocity[i] = out.new_velocity[i]
            acceleration[i] = out.new_acceleration[i]
        
        error_code = r.servo_x(position, velocity, acceleration, servox_proportional_gain)
        print(error_code) # checking if there is an error or not 
        scaling_factor = r.get_servo_trajectory_scaling_factor()
        out.pass_to_input(inp)
        time.sleep(0.001) # setting time 

    r.deactivate_servo_interface() # deactivating the servo interface

    r.stop() # stopping the robot

servo_x([-0.522, -0.319, 0.149,-3.02,-0.06,1.41]) # providing the target position for the robot
r.gripper("off") # setting gripper off condition