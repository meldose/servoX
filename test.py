from neurapy.robot import Robot 
r=Robot()

r.gripper("on")

target_angle=r.ik_fk("ik",target_pose=[0.455,-0.314,0.230,-3.02,-0.06,1,41],current_joint=[-0.322,0.211,0.263,3,0.16,-2.37])
print(target_angle)
