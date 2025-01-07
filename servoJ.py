from neurapy.robot import Robot
import time
from ruckig import InputParameter, OutputParameter, Result, Ruckig
 
r = Robot()
r.gripper("off")

def servo_j():  
    #Switch to external servo mode
    r.activate_servo_interface('position')
 
    dof = 6
 
    otg = Ruckig(dof, 0.001)  # DoFs, control cycle
    inp = InputParameter(dof)
    out = OutputParameter(dof)
 
    inp.current_position = r.get_current_joint_angles()
    inp.current_velocity = [0.]*dof
    inp.current_acceleration = [0.]*dof
 
    inp.target_position = [0.6152615188316582, -0.4936179943281102, -0.9388122207408979, 3.0132538728277254, 1.6631441804089624, -2.3839833210998176]
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

        res = otg.update(inp, out)

        position = out.new_position
        velocity = out.new_velocity
        acceleration = out.new_acceleration
 
        error_code = r.servo_j(position, velocity, acceleration)
        print(error_code)
        scaling_factor = r.get_servo_trajectory_scaling_factor()
        out.pass_to_input(inp)
        time.sleep(0.001)

    r.deactivate_servo_interface()
 
    r.stop()

servo_j()
r.gripper("on")
r.gripper("off")
