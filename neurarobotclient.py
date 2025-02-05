import time # importing time module
import logging # importing logging module
import copy # importing copy module
import json # importing json module
from neurapy.robot import Robot
from ruckig import InputParameter, OutputParameter, Result, Ruckig
import CommunicationLibrary # importing CommunicationLibrary module

CONTROLLER_IP = "192.168.1.5" # robotic controller ip
PORT = 11003 # port id number

r = Robot() # defining the robot 


def test_ls(): # defning the function for locator studio
    robot = CommunicationLibrary.RobotRequestResponseCommunication()
    try:
        logging.info(f"Connecting to robot at {CONTROLLER_IP}:{PORT}")
        robot.connect_to_server(CONTROLLER_IP, PORT) # connecting the robot with controller ip and port

        robot.pho_request_start_solution(252) # starting the solution
        robot.pho_request_ls_scan(1) # requesting the locator studio
        robot.pho_ls_wait_for_scan() # waiting for the scan
        robot.pho_request_get_objects(1, 5) # request for getting the objects
        time.sleep(0.1) # setting the time sleep
        object_coords = extract_object_coordinates(robot) # extract the object coordiantes of the robot
        if object_coords:
            formatted_coords = format_coordinates(object_coords)
            if formatted_coords:
                logging.info(f"Formatted Object Coordinates: {formatted_coords}")
                return formatted_coords # return the formatted coordinates
            else:
                logging.warning("Failed to format object coordinates.")
                return None
        else:
            logging.warning("No object coordinates found in the response.")
            return None

    except Exception as e:
        logging.error(f"Error in test_ls: {e}")
        return None
    finally:
        robot.close_connection() # close the connection with the robot

def extract_object_coordinates(robot): # definig the fucntion for the object coordinates
    try:
        objects = robot.response_data.objects
        if not objects: # if no objects found 
            logging.info("No objects found.") #s output as no object is found
            return None #return as None
        return objects[0].coordinates
    except AttributeError:
        logging.error("Invalid response structure.") # invalid response structure
        return None
    except Exception as e:
        logging.error(f"Error extracting object coordinates: {e}")
        return None

def format_coordinates(coords_mm): # definig the fucntion for coordinates
    try:
        return [x / 1000.0 for x in coords_mm] # return coordinates
    except TypeError:
        logging.error("Invalid type for coordinates.") # invalid type of coordinates 
        return None # return None

def servo_x(target_position): # defining the function for servox having target postion 
    r.activate_servo_interface('position') # activating the servo interface with position
    cart_pose_length = 7 # providing quaternion values
    otg = Ruckig(cart_pose_length, 0.001)
    inp = InputParameter(cart_pose_length)
    out = OutputParameter(cart_pose_length)

    inp.current_position = r.get_current_cartesian_pose() # get current position
    inp.current_velocity = [0.0] * cart_pose_length # get current velocity
    inp.current_acceleration = [0.0] * cart_pose_length # get current acceleration
    
    if len(target_position) == 3: # if lenght of the target postion is 3 then
        target = inp.current_position[:3] # checking the values upto 3rd postion
        target[:3] = target_position # if it matches
    else:
        target = target_position

    inp.target_position = target # if the target position is equal to the target
    inp.target_velocity = [0.0] * cart_pose_length
    inp.target_acceleration = [0.0] * cart_pose_length

    inp.max_velocity = [0.5] * cart_pose_length
    inp.max_acceleration = [3] * cart_pose_length
    inp.max_jerk = [10.0] * cart_pose_length

    res = Result.Working
    servox_proportional_gain = 25 # setting the servox propotional gain is 25
    velocity = [0.0] * 6 # setting the velocity 
    acceleration = [0.0] * 6 # setting the acceleration

    while res == Result.Working:
        res = otg.update(inp, out)
        position = out.new_position
        
        for i in range(3):
            velocity[i] = out.new_velocity[i]
            acceleration[i] = out.new_acceleration[i]
        
        error_code = r.servo_x(position, velocity, acceleration, servox_proportional_gain)
        print(f"Error Code: {error_code}") # print the error code 
        out.pass_to_input(inp)
        time.sleep(0.001) # setting the time sleep 

    r.deactivate_servo_interface() # deactivating the servo interface
    r.stop() # stopping the robot 

# Main Execution
object_coords = test_ls()
if object_coords: #if the object coordinates
    servo_x(object_coords) # calling the servox function

