
########## TESTING SCRIPT ######################################

from neurapy.robot import Robot
import time

r = Robot()

r.set_mode("Automatic") # set the mode to automatic
r.set_override(0.7) # set the override to 0.7
r.gripper("on") #   set the gripper on
target_pose=[0.7115446703604364, -0.5606851396667548, -1.1037068134297214, 3.009005155633082, 1.445595255220341, -2.2584055946550365] # object pose values in radians
linear_property = {
    "target_joint": target_pose,
}
r.move_joint(**linear_property)
r.gripper("off") # set the gripper off
