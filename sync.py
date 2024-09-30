# from neurapy.robot import Robot
import time

# Sorting robot and picking robot initialization
sorting_robot = robot1
picking_robot = robot2

conveyor_speed = 0.2  # Conveyor speed in meters per second
detection_time = time.time()  # Time when the object was detected

# Activate servo interfaces for both robots
sorting_robot.activate_servo_interface('position')
picking_robot.activate_servo_interface('position')

# Retrieve the initial Cartesian positions of both robots
sorting_pose = sorting_robot.get_current_cartesian_pose()
picking_pose = picking_robot.get_current_cartesian_pose()

# Assume object detected at a position on the conveyor
detected_position = [0.5, 0.3, 0.2]  # Example coordinates

# Sorting robot tracks and sorts the object
while True:
    current_time = time.time()
    conveyor_position_offset = conveyor_speed * (current_time - detection_time)
    updated_sorting_position = detected_position[:]
    updated_sorting_position[1] += conveyor_position_offset  # Update Y-coordinate
    
    sorting_robot.movelinear_online(updated_sorting_position, velocity=[0.15]*7, acceleration=[2.]*7)
    
    # Simulate sorting action (e.g., placing in bins)
    if sorting_robot.reaches_target_position(updated_sorting_position):
        print("Object sorted")
        break

# Picking robot starts tracking the sorted object
while True:
    current_time = time.time()
    conveyor_position_offset = conveyor_speed * (current_time - detection_time)
    updated_picking_position = updated_sorting_position[:]
    updated_picking_position[1] += conveyor_position_offset  # Update Y-coordinate
    
    picking_robot.movelinear_online(updated_picking_position, velocity=[0.15]*7, acceleration=[2.]*7)
    
    # Simulate picking action
    if picking_robot.reaches_target_position(updated_picking_position):
        print("Object picked")
        break

# Deactivate the servo interfaces after work is done
sorting_robot.stop_movelinear_online()
picking_robot.stop_movelinear_online()
sorting_robot.deactivate_servo_interface()
picking_robot.deactivate_servo_interface()
sorting_robot.stop()
picking_robot.stop()
