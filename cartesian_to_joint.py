from neurapy.robot import Robot
r = Robot()
target_angle = r.ik_fk("ik", target_pose =[-332.404,211.217,263.136, 3.0,0.16,-2.37], # object pose values in radians
current_joint = [-0.456,-0.314,0.210,-3.02,-0.06,1.41]) # current joint angles in radians
print(target_angle)
