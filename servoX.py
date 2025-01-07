from neurapy.robot import Robot
import time
from ruckig import InputParameter, OutputParameter, Result, Ruckig
import copy

def servo_x(self,*args,**kwargs):
    
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
    inp.target_position = [-0.522, -0.319, 0.149,-3.02,-0.06,1.41]
    inp.target_velocity = [0.]*cart_pose_length
    inp.target_acceleration = [0.]*cart_pose_length

    inp.max_velocity = [0.5]*cart_pose_length
    inp.max_acceleration = [3]*cart_pose_length
    inp.max_jerk = [10.]*cart_pose_length
    res = Result.Working

    servox_proportional_gain = 25

    velocity = [0.] * 6 #Since ruckig does not provide rotational velocity if quaternion is input, we can send 0 rotational feedforward velocity
    acceleration = [0.] * 6 #Since ruckig does not provide rotational acceleration if quaternion is input, we can send 0 rotational feedforward acceleration

    while res == Result.Working:
        error_code = 0
        # if(error_code < 3):

        res = otg.update(inp, out)

        position = out.new_position

        for i in range(0,3): # Updating target translation velocity and accelerations
            velocity[i] = out.new_velocity[i]
            acceleration[i] = out.new_acceleration[i]
        
        error_code = r.servo_x(position, velocity, acceleration, servox_proportional_gain)
        print(error_code)
        scaling_factor = r.get_servo_trajectory_scaling_factor()
        out.pass_to_input(inp)
        time.sleep(0.001)
        # else:
        #     print("Servo in error, error code, ", error_code)
        #     break
    r.deactivate_servo_interface()

    r.stop()

servo_x()
    