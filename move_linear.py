from neurapy.robot import Robot
import time
import copy

r = Robot()



def movelinear_online(self,*args,**kwargs):
    target_1 = 0.3
    target_2 = 0.25
 
    #Switch to external servo mode
    r.activate_servo_interface('position')
    cart_pose_length = 7 # X,Y,Z,qw,qx,qy,qz
    target = copy.deepcopy(r.get_current_cartesian_pose())
    print(target)

    # Move target_1 unit in -X direction
    target[0] -= target_1
    velocity = [0.15]*6 
    acceleration = [2.]*6
    error_code = r.movelinear_online(target, velocity, acceleration)

    #Sleep for 5 sec to complete the motion.
    time.sleep(5)

    target = copy.deepcopy(r.get_current_cartesian_pose())
    #Move target_2 units in +Z direction
    target[2] += target_2
    error_code = r.movelinear_online(target, velocity, acceleration)

    #Sleep for 5 sec to complete the motion
    time.sleep(5)

    print("Robot stopped")
    r.deactivate_servo_interface()
    r.stop()

    self.logger.info(
            "MOVELINEAR called with parameters {} {}".format(args, kwargs)
        )
    command = Servo(self)
    command.execute_visual_servoing(*args,**kwargs)