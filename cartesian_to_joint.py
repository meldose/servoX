from neurapy.robot import Robot
r = Robot()
target_angle = r.ik_fk("ik", target_pose =[-0.456, -0.315, 0.120,-3.04,-1.24,0.53], # object pose values in radians
current_joint = [-0.456,-0.314,0.210,-3.02,-0.06,1.41]) # current joint angles in radians
print(target_angle)
