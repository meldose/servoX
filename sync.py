import time # import time module
from neurapy.robot import Robot  # Assuming you have this import

# Initialize robots
sorting_robot = Robot("robot1") # seting sorting robot
picking_robot = Robot("robot2") # seting picking robot

conveyor_speed = 0.2  # Conveyor speed in meters per second
detection_time = time.time()  # Time when the object was detected

# Activate servo interfaces for both robots
sorting_robot.activate_servo_interface('position') # seting sorting robot
picking_robot.activate_servo_interface('position') # seting picking robot

# Retrieve the initial Cartesian positions of both robots
sorting_pose = sorting_robot.get_current_cartesian_pose() # seting sorting pose
picking_pose = picking_robot.get_current_cartesian_pose() # seting picking pose
 
# Assume object detected at a position on the conveyor
detected_position = [0.5, 0.3, 0.2]  # Example coordinates # seting detected position

def sort_object(): # defining sorting function
    while True: # while loop
        current_time = time.time() # set current time
        conveyor_position_offset = conveyor_speed * (current_time - detection_time) # equation for conveyor position offset
        updated_sorting_position = detected_position[:] # update sorting position
        updated_sorting_position[1] += conveyor_position_offset  # Update Y-coordinate 
        
        sorting_robot.movelinear_online(updated_sorting_position, velocity=[0.15]*7, acceleration=[2.]*7) # moving the sorting robot to the updated position
        
        if sorting_robot.reaches_target_position(updated_sorting_position): # check if the sorting robot reaches the updated position
            print("Object sorted") # print message
            return updated_sorting_position  # Return the position for the picking robot

def pick_object(updated_sorting_position): # defining picking function
    while True: # while loop
        current_time = time.time() # set current time
        conveyor_position_offset = conveyor_speed * (current_time - detection_time) # equation for conveyor position offset
        updated_picking_position = updated_sorting_position[:] # update picking position
        updated_picking_position[1] += conveyor_position_offset  # Update Y-coordinate # seting updated picking position
        
        picking_robot.movelinear_online(updated_picking_position, velocity=[0.15]*7, acceleration=[2.]*7)
        
        if picking_robot.reaches_target_position(updated_picking_position): # checkif the picking robot reaches the updated position
            print("Object picked") # print message
            return

# Sorting and picking process
sorted_position = sort_object() # call sorting function
pick_object(sorted_position) # call picking function

# Deactivate the servo interfaces after work is done
sorting_robot.stop_movelinear_online() #stop movelinear_online for sorting robot
picking_robot.stop_movelinear_online() # stop movelinear_online for picking robot
sorting_robot.deactivate_servo_interface() # deactivate servo interface for sorting robot
picking_robot.deactivate_servo_interface() # deactivate servo interface for picking robot
sorting_robot.stop() # stop sorting robot
picking_robot.stop() # stop picking robot