from ...commands.import_commands import *

def servo_j(self,*args,**kwargs):
    
    """
    Method to do Servo in Joint Space
    
    Args:
    - servo_target_position ([int]*dof)     : List of joint target position in rad. Length of list should be equal to degree of freedom
    - servo_target_velocity ([int]*dof)     : List of joint target velocity in rad/s. Length of list should be equal to degree of freedom
    - servo_target_acceleration ([int]*dof) : List of joint target acceleration in rad/s^2. Length of list should be equal to degree of freedom

    Returns:
    - Servo warning and error codes

    **Sample Usage**
    
        .. code-block:: python

            from neurapy.robot import Robot
            import time
            from ruckig import InputParameter, OutputParameter, Result, Ruckig

            r = Robot()

            #Switch to external servo mode
            r.activate_servo_interface('position')

            dof = 6

            otg = Ruckig(dof, 0.001)  # DoFs, control cycle
            inp = InputParameter(dof)
            out = OutputParameter(dof)

            inp.current_position = r.get_current_joint_angles()
            inp.current_velocity = [0.]*dof
            inp.current_acceleration = [0.]*dof

            inp.target_position = [0., 0., 0., 0., 0., 0.]
            inp.target_velocity = [0.]*dof
            inp.target_acceleration = [0.]*dof

            inp.max_velocity = [0.5]*dof
            inp.max_acceleration = [3]*dof
            inp.max_jerk = [10.]*dof
            res = Result.Working



            while res == Result.Working:
                '''
                Error code is returned through Servo. 
                '''
                error_code = 0
                if(error_code < 3):

                    res = otg.update(inp, out)

                    position = out.new_position
                    velocity = out.new_velocity 
                    acceleration = out.new_acceleration

                    error_code = r.servo_j(position, velocity, acceleration)
                    scaling_factor = r.get_servo_trajectory_scaling_factor()
                    out.pass_to_input(inp)
                    time.sleep(0.001)
                else:
                    print("Servo in error, error code, ", error_code)
                    break
            r.deactivate_servo_interface()

            r.stop()
    """
    
    self.logger.info(
            "ServoJ called with parameters {} {}".format(args, kwargs)
        )
    
    command = Servo(self)
    add_additional_argument(command, *args, additional_args=(0), **kwargs)


def servo_x(self,*args,**kwargs):
    
    """
    Method to do Servo in Cartesian Space
    
    Args:
    - servo_target_position ([int]*7)     : List of cartesian target position in m. Length of list should be equal to 7 (X, Y, Z, qw, qx, qz, qz)
    - servo_target_velocity ([int]*7)     : List of cartesian target velocity in m/s. Length of list should be equal to 7
    - servo_target_acceleration ([int]*7) : List of cartesian target acceleration in m/s^2. Length of list should be equal 7. Not used in Current Version
    - gain parameter (double)             : ServoX gain parameter, used for Propotional controller (should be between [0.2, 100])

    Returns:
    - Servo warning and error codes

    **Sample Usage**
    
        .. code-block:: python

            from neurapy.robot import Robot
            import time
            from ruckig import InputParameter, OutputParameter, Result, Ruckig
            import copy

            r = Robot()

            #Switch to external servo mode
            r.activate_servo_interface('position')

            cart_pose_length = 7 #X,Y,Z,qw,qx,qy,qz

            otg = Ruckig(cart_pose_length, 0.001)  # control cycle
            inp = InputParameter(cart_pose_length)
            out = OutputParameter(cart_pose_length)

            inp.current_position = r.get_current_cartesian_pose()
            inp.current_velocity = [0.]*cart_pose_length
            inp.current_acceleration = [0.]*cart_pose_length

            target = copy.deepcopy(inp.current_position)
            target[0] += 0.2 # Move 200mm in X direction
            inp.target_position = target
            inp.target_velocity = [0.]*cart_pose_length
            inp.target_acceleration = [0.]*cart_pose_length

            inp.max_velocity = [0.5]*cart_pose_length
            inp.max_acceleration = [3]*cart_pose_length
            inp.max_jerk = [10.]*cart_pose_length
            res = Result.Working

            servox_proportional_gain = 25

            while res == Result.Working:
                '''
                Error code is returned through Servo. 
                '''
                error_code = 0
                if(error_code < 3):

                    res = otg.update(inp, out)

                    position = out.new_position
                    velocity = out.new_velocity 
                    acceleration = out.new_acceleration

                    error_code = r.servo_x(position, velocity, acceleration, servox_proportional_gain)
                    scaling_factor = r.get_servo_trajectory_scaling_factor()
                    out.pass_to_input(inp)
                    time.sleep(0.001)
                else:
                    print("Servo in error, error code, ", error_code)
                    break
            r.deactivate_servo_interface()

            r.stop()
    """
    
    self.logger.info(
            "ServoX called with parameters {} {}".format(args, kwargs)
        )
    
    command = Servo(self)
    add_additional_argument(command, *args, additional_args=(1), **kwargs)

def movelinear_online(self,*args,**kwargs):
    
    """
    Method to do Servo in Cartesian Space
    
    Args:
    - servo_target_position ([int]*7)     : List of cartesian target position in m. Length of list should be equal to 7 (X, Y, Z, qw, qx, qz, qz)
    - servo_target_velocity ([int]*7)     : List of cartesian target velocity in m/s. Length of list should be equal to 7
    - servo_target_acceleration ([int]*7) : List of cartesian target acceleration in m/s^2. Length of list should be equal 7. Not used in Current Version
    - gain parameter (double)             : ServoX gain parameter, used for Propotional controller (should be between [0.2, 100])

    Returns:
    - Servo warning and error codes

    **Sample Usage**
    
        .. code-block:: python

            from neurapy.robot import Robot
            import time
            from ruckig import InputParameter, OutputParameter, Result, Ruckig
            import copy

            r = Robot()

            #Switch to external servo mode
            r.activate_servo_interface('position')

            cart_pose_length = 7 #X,Y,Z,qw,qx,qy,qz

            otg = Ruckig(cart_pose_length, 0.001)  # control cycle
            inp = InputParameter(cart_pose_length)
            out = OutputParameter(cart_pose_length)

            inp.current_position = r.get_current_cartesian_pose()
            inp.current_velocity = [0.]*cart_pose_length
            inp.current_acceleration = [0.]*cart_pose_length

            target = copy.deepcopy(inp.current_position)
            target[0] += 0.2 # Move 200mm in X direction
            inp.target_position = target
            inp.target_velocity = [0.]*cart_pose_length
            inp.target_acceleration = [0.]*cart_pose_length

            inp.max_velocity = [0.5]*cart_pose_length
            inp.max_acceleration = [3]*cart_pose_length
            inp.max_jerk = [10.]*cart_pose_length
            res = Result.Working

            servox_proportional_gain = 25

            while res == Result.Working:
                '''
                Error code is returned through Servo. 
                '''
                error_code = 0
                if(error_code < 3):

                    res = otg.update(inp, out)

                    position = out.new_position
                    velocity = out.new_velocity 
                    acceleration = out.new_acceleration

                    error_code = r.servo_x(position, velocity, acceleration, servox_proportional_gain)
                    scaling_factor = r.get_servo_trajectory_scaling_factor()
                    out.pass_to_input(inp)
                    time.sleep(0.001)
                else:
                    print("Servo in error, error code, ", error_code)
                    break
            r.deactivate_servo_interface()

            r.stop()
    """
    
    self.logger.info(
            "MOVELINEAR called with parameters {} {}".format(args, kwargs)
        )
    if("maira" in self.robot_name.lower()):
        self.robot.logger.warning("MOVELINEAR not implemented for Maira. Stay tuned for updates.")
        return False
    
    command = Servo(self)
    command.execute_visual_servoing(*args,**kwargs)


def get_servo_trajectory_scaling_factor(self, *args, **kwargs):
    
    """
    Method to get the trajectory scaling factor from ServoJ

    Returns:
    - [double] scaling factor

    **Sample Usage**
    
        .. code-block:: python
        
            from neurapy.robot import Robot
            import time
            from ruckig import InputParameter, OutputParameter, Result, Ruckig

            r = Robot()

            #Switch to external servo mode
            r.activate_servo_interface('position')

            dof = 6

            otg = Ruckig(dof, 0.001)  # DoFs, control cycle
            inp = InputParameter(dof)
            out = OutputParameter(dof)

            inp.current_position = r.get_current_joint_angles()
            inp.current_velocity = [0.]*dof
            inp.current_acceleration = [0.]*dof

            inp.target_position = [0., 0., 0., 0., 0., 0.]
            inp.target_velocity = [0.]*dof
            inp.target_acceleration = [0.]*dof

            inp.max_velocity = [0.5]*dof
            inp.max_acceleration = [3]*dof
            inp.max_jerk = [10.]*dof
            res = Result.Working



            while res == Result.Working:
                '''
                Error code is returned through Servo. 
                '''
                error_code = 0
                if(error_code < 3):

                    res = otg.update(inp, out)

                    position = out.new_position
                    velocity = out.new_velocity 
                    acceleration = out.new_acceleration

                    error_code = r.servo_j(position, velocity, acceleration)
                    scaling_factor = r.get_servo_trajectory_scaling_factor()
                    out.pass_to_input(inp)
                    time.sleep(0.001)
                else:
                    print("Servo in error, error code, ", error_code)
                    break
            r.deactivate_servo_interface()

            r.stop()
    """
    command = Servo(self)
    return command.get_trajectory_scaling_factor()

#Additional function to handle servox and servoj last argument. 0 is for servoj, 1 is for servox, 2 is for servox program
def add_additional_argument(func, *args, additional_args=(), **kwargs):

    additional_args = (additional_args,)
    new_args = args + additional_args

    func.set_parameter(*new_args,**kwargs)
    return func.execute()

def stop_movelinear_online(self, *args, **kwargs):
    if("maira" in self.robot_name.lower()):
        self.robot.logger.warning("ServoJ not implemented for Maira. Stay tuned for updates.")
        return

    command = Servo(self)
    
    command.stop_movelinear_online()