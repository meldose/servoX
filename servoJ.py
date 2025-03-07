import copy # importing copy module

class ServoJ:  # defining servoJ
    
    def __init__(self, robot):  # initializing the robot
        self.robot = robot  # setting the robot

    def servo_j(self, message): # defining the servo_j function
        # message = [x / 1000 for x in message]  # Scale values
        z_offset=0.04
        x = message[0] / 1000 # Scale values
        y = message[1] / 1000 # Scale values
        z = message[2] / 1000 + z_offset # created an offset
        w = message[3] # orientation values
        ex= message[4] # orientation values
        ey= message[5] # orientation values
        ez= message[6] # orientation values
        
        new_message = [x, y,z,w,ex,ey,ez] # added new order for quaternion values
        
        print(message)# printing the message
        
        print(new_message)# printing the new ordered message

        # Activate servo interface
        r.activate_servo_interface('position')
        dof = 6  # Degrees of freedom
        otg = Ruckig(dof, 0.001)  # Online trajectory generator
    
        quaternion_pose = new_message # [X, Y, Z, W, EX, EY, EZ]
        euler_pose = r.convert_quaternion_to_euler_pose(quaternion_pose) # getting te euler pose

        print("converted euler Angles are:", euler_pose)  # Output: [X, Y, Z, R, P, Y] with Euler angle values.

        # Input/Output parameters
        inp = InputParameter(dof) #setting the input parameter
        out = OutputParameter(dof) #setting the output parameter

        # Current state
        inp.current_position = r.get_current_joint_angles() # getting the current joint angles
        inp.current_velocity = [0.0] * dof # setting the current velocity as zero
        inp.current_acceleration = [0.0] * dof # setting the current acceleration as zero

        target_end_effector_pose = euler_pose # setting the target end effector pose
        reference_joint_angles = r.get_current_joint_angles() # getting the current joint angles
        joint_angle_solution = r.compute_inverse_kinematics(target_end_effector_pose, reference_joint_angles) # computing the inverse kinematics

        print("Target Joint Angles:", joint_angle_solution) # print the target joint angles
        
        inp.target_position = joint_angle_solution # setting the target position 
        target = copy.deepcopy(inp.current_position) # copying the current position of the robot 
        # inp.target_position = [new_message[0], new_message[1], new_message[2], target[3], target[4], target[5], target[6]] # passing the values by fixing the [X,Y,z and fixing the d,a,b,c]
        inp.target_acceleration = [0.0] * dof # setting the target acceleration as zero
        inp.max_velocity = [0.8] * dof #    defining the maximum velocity
        inp.max_acceleration = [7.0] * dof # defining the maximum acceleration
        inp.max_jerk = [5.0] * dof # defining the maximum jerk

        res = Result.Working # setting the res variable as Result.Working

        while res == Result.Working: # while the result is working
            res = otg.update(inp, out) # updating the input and output
            error_code = r.servo_j(out.new_position, out.new_velocity, out.new_acceleration) # passing the error code variable with having servo_j function having position, velocity and acceleration
            scaling_factor = r.get_servo_trajectory_scaling_factor() # getting the servo trajectory scaling factors
            out.pass_to_input(inp) # passing the output to the input
            time.sleep(0.001) # setting the time sleep to 0.001 seconds

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
