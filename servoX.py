import copy # importing copy module

class ServoX: # defining servoX

    def __init__(self,robot): # initializing the robot
        self.robot = robot # setting the robot


    def servo_x(self,message,*args,**kwargs): # defining servoX
        
      # message = [x/1000 for x in message] # converting the values to mm
        z_offset = 0.04
        x = message[0] / 1000 # Scale values
        y = message[1] / 1000 # Scale values
        z = message[2] / 1000  + z_offset  # created an offset
        w = message[3] # orientation values
        ex= message[4] # orientation values
        ey= message[5] # orientation values
        ez= message[6] # orientation values
        
        print(message) # printing the message

        new_message = [x, y,z,w,ex,ey,ez] # added new order for quaternion values
        
        print(new_message) # printing the new ordered message

        r = self.robot #setting the robot

        #Switch to external servo mode
        r.activate_servo_interface('position') # activating the servo interface

        cart_pose_length = 7 #X,Y,Z,qw,qx,qy,qz

        otg = Ruckig(cart_pose_length, 0.001)  # control cycle
        inp = InputParameter(cart_pose_length) # setting the inputparameter with cart pose length
        out = OutputParameter(cart_pose_length) # setting the outputparmeter with cart pose length

        inp.current_position = r.get_current_cartesian_pose() # getting the current cartesian poses
        inp.current_velocity = [0.]*cart_pose_length # mutliplying the initial velocity with cart pose lenght 
        inp.current_acceleration = [0.]*cart_pose_length # mutliplying the current acceleration with cart pose length

        target = copy.deepcopy(inp.current_position) # copying the current position of the robot 
        # inp.target_position = [new_message[0], new_message[1], new_message[2], target[3], target[4], target[5], target[6]]
        inp.target_position = new_message
        inp.target_position[3] += 0.03

        inp.target_velocity = [0.]*cart_pose_length # defning the target velocity
        inp.target_acceleration = [0.]*cart_pose_length # defining the target acceleration

        inp.max_velocity = [80.0]*cart_pose_length # setting the maximum velocity with 0.5 times the cart pose length
        inp.max_acceleration = [65.0]*cart_pose_length #se tting the max acceleration with 3 times the cart pose length
        inp.max_jerk = [1.0]*cart_pose_length # setting the jerk values

        servox_proportional_gain = 25 # setting the servox propotional gain as 25

        velocity = [0.] * 6 # Since ruckig does not provide rotational velocity if quaternion is input, we can send 0 rotational feedforward velocity
        acceleration = [0.] * 6 # Since ruckig does not provide rotational acceleration if quaternion is input, we can send 0 rotational feedforward acceleration
        
        res=Result.Working # setting the result

        while res == Result.Working: # while the result is working
            
            error_code = 0 # setting the error code

            res = otg.update(inp, out) # updating the input and output

            position = out.new_position # getting the new position

            # for i in range(0,3): # Updating target translation velocity and accelerations
                # velocity[i] = out.new_velocity[i]
                # acceleration[i] = out.new_acceleration[i]

            zeros = [0.] * 6 #setting the zeros

            error_code = r.servo_x(position, zeros, zeros, servox_proportional_gain) # passing the error code variable with having servo_j function having position, velocity and acceleration
            scaling_factor = r.get_servo_trajectory_scaling_factor() # getting the servo trajectory scaling factors
            out.pass_to_input(inp)
            time.sleep(0.00000000002) # setting time 
            
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