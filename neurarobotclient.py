import time # import time module
import logging # import logging module  
import copy # import copy module
import json # import json module
from neurapy.robot import Robot
from ruckig import InputParameter, OutputParameter, Result, Ruckig
import CommunicationLibrary # import CommunicationLibrary

CONTROLLER_IP = "192.168.1.5" # robotic controller ip
PORT = 11003 # port id number

r = Robot() # defining the robot 


def test_ls(): # defining the function for locator studio
    robot = CommunicationLibrary.RobotRequestResponseCommunication()
    try:
        logging.info(f"Connecting to robot at {CONTROLLER_IP}:{PORT}")
        robot.connect_to_server(CONTROLLER_IP, PORT) # settig the connection with server having controller ip and Port

        robot.pho_request_start_solution(252) # setting start solution
        robot.pho_request_ls_scan(1) # requesting the locator scan
        robot.pho_ls_wait_for_scan() # waiting for the scan
        robot.pho_request_get_objects(1, 5) # reuqesting the objects
        time.sleep(0.1) # setting the time sleep 
        object_coords = extract_object_coordinates(robot)
        if object_coords:
            formatted_coords = format_coordinates(object_coords)
            if formatted_coords: # if the formatted coordinates are there then 
                logging.info(f"Formatted Object Coordinates: {formatted_coords}")
                return formatted_coords # return the formatted coordinates
            else:
                logging.warning("Failed to format object coordinates.")
                return None # return None
        else:
            logging.warning("No object coordinates found in the response.")
            return None # return None 

    except Exception as e:
        logging.error(f"Error in test_ls: {e}")
        return None # return None
    finally:
        robot.close_connection() # close the robot connection
        
def extract_object_coordinates(robot): # definig the robot fucntion for object coordinates
    try:
        objects = robot.response_data.objects
        if not objects: # if not object 
            logging.info("No objects found.") # no objects found
            return None # return None
        return objects[0].coordinates
    except AttributeError:
        logging.error("Invalid response structure.") # Invalid response structure
        return None # return None
    except Exception as e:
        logging.error(f"Error extracting object coordinates: {e}")
        return None

def format_coordinates(coords_mm): # defining the function for format coordinates
    try:
        return [x / 1000.0 for x in coords_mm] # retunr the coordinates values in mm
    except TypeError:
        logging.error("Invalid type for coordinates.")
        return None

def servo_x(target_position): # defining the servox function
    r.activate_servo_interface('position') # activating the servo interface
    cart_pose_length = 7 # settign the quaternion values
    otg = Ruckig(cart_pose_length, 0.001)
    inp = InputParameter(cart_pose_length)
    out = OutputParameter(cart_pose_length)

    inp.current_position = r.get_current_cartesian_pose() # getting the current cartesian pose
    inp.current_velocity = [0.0] * cart_pose_length # setting the velocity
    inp.current_acceleration = [0.0] * cart_pose_length # setting the acceleration
    
    if len(target_position) == 3: # if lenght of the target position is equal to 3 or not
        target = inp.current_position[:3] # if target value is set up to the current position checking with the 3rd position
        target[:3] = target_position # setting the target third position with the target position
    else:
        target = target_position

    inp.target_position = target
    inp.target_velocity = [0.0] * cart_pose_length # setting the target velocity
    inp.target_acceleration = [0.0] * cart_pose_length # setting the target acceleration

    inp.max_velocity = [0.5] * cart_pose_length # settng the max velocity
    inp.max_acceleration = [3] * cart_pose_length # setting the max acceleration
    inp.max_jerk = [10.0] * cart_pose_length # setting the max jerk

    res = Result.Working
    servox_proportional_gain = 25
    velocity = [0.0] * 6
    acceleration = [0.0] * 6

    while res == Result.Working:
        res = otg.update(inp, out)
        position = out.new_position
        
        for i in range(3):
            velocity[i] = out.new_velocity[i]
            acceleration[i] = out.new_acceleration[i]
        
        error_code = r.servo_x(position, velocity, acceleration, servox_proportional_gain)
        print(f"Error Code: {error_code}") # print the error code 
        out.pass_to_input(inp)
        time.sleep(0.001) # setting up the time sleep

    r.deactivate_servo_interface() # deactivating the servo interface
    r.stop() # stop the robot

# Main Execution
object_coords = test_ls() # setting the object coords with function test locator
if object_coords: # if the object coordinates 
    servo_x(object_coords) # cal the servox function

