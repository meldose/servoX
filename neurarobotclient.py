# import time
# import logging
# import copy
# import json
# from neurapy.robot import Robot
# from ruckig import InputParameter, OutputParameter, Result, Ruckig
# import CommunicationLibrary

# CONTROLLER_IP = "192.168.1.5" # robotic controller ip
# PORT = 11003 # port id number

# r = Robot() # defining the robot 


# def test_ls():
#     robot = CommunicationLibrary.RobotRequestResponseCommunication()
#     try:
#         logging.info(f"Connecting to robot at {CONTROLLER_IP}:{PORT}")
#         robot.connect_to_server(CONTROLLER_IP, PORT)

#         robot.pho_request_start_solution(252)
#         robot.pho_request_ls_scan(1)
#         robot.pho_ls_wait_for_scan()
#         robot.pho_request_get_objects(1, 5)
#         time.sleep(0.1)
#         object_coords = extract_object_coordinates(robot)
#         if object_coords:
#             formatted_coords = format_coordinates(object_coords)
#             if formatted_coords:
#                 logging.info(f"Formatted Object Coordinates: {formatted_coords}")
#                 return formatted_coords
#             else:
#                 logging.warning("Failed to format object coordinates.")
#                 return None
#         else:
#             logging.warning("No object coordinates found in the response.")
#             return None

#     except Exception as e:
#         logging.error(f"Error in test_ls: {e}")
#         return None
#     finally:
#         robot.close_connection()

# def extract_object_coordinates(robot):
#     try:
#         objects = robot.response_data.objects
#         if not objects:
#             logging.info("No objects found.")
#             return None
#         return objects[0].coordinates
#     except AttributeError:
#         logging.error("Invalid response structure.")
#         return None
#     except Exception as e:
#         logging.error(f"Error extracting object coordinates: {e}")
#         return None

# def format_coordinates(coords_mm):
#     try:
#         return [x / 1000.0 for x in coords_mm]
#     except TypeError:
#         logging.error("Invalid type for coordinates.")
#         return None

# def servo_x(target_position):
#     r.activate_servo_interface('position')
#     cart_pose_length = 7
#     otg = Ruckig(cart_pose_length, 0.001)
#     inp = InputParameter(cart_pose_length)
#     out = OutputParameter(cart_pose_length)

#     inp.current_position = r.get_current_cartesian_pose()
#     inp.current_velocity = [0.0] * cart_pose_length
#     inp.current_acceleration = [0.0] * cart_pose_length
    
#     if len(target_position) == 3:
#         target = inp.current_position[:3]
#         target[:3] = target_position
#     else:
#         target = target_position

#     inp.target_position = target
#     inp.target_velocity = [0.0] * cart_pose_length
#     inp.target_acceleration = [0.0] * cart_pose_length

#     inp.max_velocity = [0.5] * cart_pose_length
#     inp.max_acceleration = [3] * cart_pose_length
#     inp.max_jerk = [10.0] * cart_pose_length

#     res = Result.Working
#     servox_proportional_gain = 25
#     velocity = [0.0] * 6
#     acceleration = [0.0] * 6

#     while res == Result.Working:
#         res = otg.update(inp, out)
#         position = out.new_position
        
#         for i in range(3):
#             velocity[i] = out.new_velocity[i]
#             acceleration[i] = out.new_acceleration[i]
        
#         error_code = r.servo_x(position, velocity, acceleration, servox_proportional_gain)
#         print(f"Error Code: {error_code}")
#         out.pass_to_input(inp)
#         time.sleep(0.001)

#     r.deactivate_servo_interface()
#     r.stop()

# # Main Execution
# object_coords = test_ls()
# if object_coords:
#     servo_x(object_coords)

import time
import logging
import copy
from neurapy.robot import Robot
from ruckig import InputParameter, OutputParameter, Result, Ruckig
import CommunicationLibrary

# Setup logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

CONTROLLER_IP = "192.168.1.5"  # Robotic controller IP
PORT = 11003  # Port ID

# Define Robot
r = Robot()

def test_ls():
    """Performs laser scan and extracts object coordinates."""
    robot = CommunicationLibrary.RobotRequestResponseCommunication()
    try:
        logging.info(f"Connecting to robot at {CONTROLLER_IP}:{PORT}")
        robot.connect_to_server(CONTROLLER_IP, PORT)

        logging.info("Starting laser scan...")
        robot.pho_request_start_solution(252)
        robot.pho_request_ls_scan(1)
        robot.pho_ls_wait_for_scan()
        robot.pho_request_get_objects(1, 5)

        time.sleep(0.1)

        object_coords = extract_object_coordinates(robot)
        if object_coords:
            formatted_coords = format_coordinates(object_coords)
            if formatted_coords:
                logging.info(f"Formatted Object Coordinates: {formatted_coords}")
                return formatted_coords
            else:
                logging.warning("Failed to format object coordinates.")
        else:
            logging.warning("No object coordinates found in the response.")

        return None

    except Exception as e:
        logging.error(f"Error in test_ls: {e}")
        return None
    finally:
        robot.close_connection()
        logging.info("Robot connection closed.")

def extract_object_coordinates(robot):
    """Extracts object coordinates from the robot's response."""
    try:
        objects = getattr(robot.response_data, "objects", None)
        if not objects or len(objects) == 0:
            logging.info("No objects found.")
            return None

        return objects[0].coordinates

    except AttributeError as e:
        logging.error(f"Invalid response structure: {e}")
        return None
    except Exception as e:
        logging.error(f"Error extracting object coordinates: {e}")
        return None

def format_coordinates(coords_mm):
    """Converts coordinates from millimeters to meters."""
    try:
        if not isinstance(coords_mm, list):
            raise TypeError("Coordinates must be a list.")
        return [x / 1000.0 for x in coords_mm]
    except TypeError as e:
        logging.error(f"Invalid type for coordinates: {e}")
        return None

def servo_j():
    """Moves the robot using joint servo control."""
    logging.info("Activating servo interface in position mode.")
    r.activate_servo_interface("position")
    
    dof = 6  # Degrees of freedom
    otg = Ruckig(dof, 0.001)

    inp = InputParameter(dof)
    out = OutputParameter(dof)

    inp.current_position = r.get_current_joint_angles()
    inp.current_velocity = [0.0] * dof
    inp.current_acceleration = [0.0] * dof

    inp.target_position = [1.1625650370244778, -0.5774947959093657, -1.6300017754314295,
                           1.9807964651163987, 1.5676122261006906, 0.636066807616557]
    
    inp.target_velocity = [0.0] * dof
    inp.target_acceleration = [0.0] * dof

    inp.max_velocity = [0.5] * dof
    inp.max_acceleration = [3.0] * dof
    inp.max_jerk = [10.0] * dof

    res = Result.Working

    logging.info("Starting joint motion...")

    while res == Result.Working:
        res = otg.update(inp, out)

        position = out.new_position
        velocity = out.new_velocity
        acceleration = out.new_acceleration

        error_code = r.servo_j(position, velocity, acceleration)
        if error_code:
            logging.warning(f"Servo error code: {error_code}")

        scaling_factor = r.get_servo_trajectory_scaling_factor()
        out.pass_to_input(inp)
        time.sleep(0.001)

    logging.info("Deactivating servo interface and stopping robot.")
    r.deactivate_servo_interface()
    r.stop()

# Main Execution
if __name__ == "__main__":
    r.gripper("on")
    object_coords = test_ls()
    if object_coords:
        servo_j()
    else:
        logging.warning("No valid object coordinates received. Skipping servo motion.")
    r.gripper("off")
