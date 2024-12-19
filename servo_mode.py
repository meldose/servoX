
# from neurapy.robot import Robot
# import time
# from ruckig import InputParameter, OutputParameter, Result, Ruckig
# import copy

# r = Robot()
    

# def movelinear_online():
#     #Switch to external servo mode
#     r.activate_servo_interface('position')

#     cart_pose_length = 7 #X,Y,Z,qw,qx,qy,qz

#     otg = Ruckig(cart_pose_length, 0.001)  # control cycle
#     inp = InputParameter(cart_pose_length)
#     out = OutputParameter(cart_pose_length)

#     inp.current_position = r.get_current_cartesian_pose()
#     inp.current_velocity = [0.]*cart_pose_length
#     inp.current_acceleration = [0.]*cart_pose_length

#     target = copy.deepcopy(inp.current_position)
#     target[0] += 0.2 # Move 200mm in X direction
#     inp.target_position = target
#     inp.target_velocity = [0.]*cart_pose_length
#     inp.target_acceleration = [0.]*cart_pose_length

#     inp.max_velocity = [0.5]*cart_pose_length
#     inp.max_acceleration = [3]*cart_pose_length
#     inp.max_jerk = [10.]*cart_pose_length
#     res = Result.Working

#     servox_proportional_gain = 25

#     while res == Result.Working:
#         '''
#         Error code is returned through Servo. 
#         '''
#         error_code = 0
#         if(error_code < 3):

#             res = otg.update(inp, out)

#             position = out.new_position
#             velocity = out.new_velocity 
#             acceleration = out.new_acceleration

#             error_code = r.servo_x(position, velocity, acceleration, servox_proportional_gain)
#             scaling_factor = r.get_servo_trajectory_scaling_factor()
#             out.pass_to_input(inp)
#             time.sleep(0.001)
#         else:
#             print("Servo in error, error code, ", error_code)
#             break
#     r.deactivate_servo_interface()

#     r.stop()

# movelinear_online()



from neurapy.robot import Robot
import time
from ruckig import InputParameter, OutputParameter, Result, Ruckig
import copy

r = Robot()
r.set_override(0.4)
# r.set_mode("Automatic")
def movelinear_online():
    #Switch to external servo mode
    r.activate_servo_interface('position')

    cart_pose_length = 7  # X, Y, Z, qw, qx, qy, qz

    otg = Ruckig(cart_pose_length, 0.001)  # control cycle
    inp = InputParameter(cart_pose_length)
    out = OutputParameter(cart_pose_length)

    # Set current position (your provided coordinates)
    inp.current_position = [-0.533, -0.417, 0.234, 0.161, 0.804, -0.525, 0.227]
    inp.current_velocity = [0.] * cart_pose_length
    inp.current_acceleration = [0.] * cart_pose_length

    # Target position (displace 200mm in X direction)
    target = copy.deepcopy(inp.current_position)
    target[0] += 0.2  # Move 200mm in the X direction (add to X component)
    inp.target_position = target
    inp.target_velocity = [0.] * cart_pose_length
    inp.target_acceleration = [0.] * cart_pose_length

    # Motion limits
    inp.max_velocity = [0.5] * cart_pose_length
    inp.max_acceleration = [3] * cart_pose_length
    inp.max_jerk = [10.] * cart_pose_length
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

movelinear_online()
