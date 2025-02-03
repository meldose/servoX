from neurapy.robot import Robot
r = Robot()
target_angle = r.ik_fk("ik", target_pose =[-0.460,-0.339,0.210,-3.02,-0.06,1.41], # object pose values in radians
current_joint = [-0.419,0.129,0.199,3,0.54,-1.94]) # current joint angles in radians
print(target_angle)

