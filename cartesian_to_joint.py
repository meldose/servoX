from neurapy.robot import Robot
r = Robot()
target_angle = r.ik_fk("ik", target_pose =[-0.456,-0.314,0.230,-3.02,-0.06,1.41], # object pose values in radians
#current_joint = [-0.332,0.211,0.263,3,0.16,-2.37])
current_joint = [-0.288,0.180,0.280,3.02,0.096,-2.654]) # current joint angles in radians
print(target_angle)
