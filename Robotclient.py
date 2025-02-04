#!/usr/bin/env python3

import CommunicationLibrary
import time
import json
import logging

CONTROLLER_IP = "192.168.1.5"
PORT = 11003


def test_ls():
    robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
    robot.connect_to_server(CONTROLLER_IP,PORT)  # communication between VC and robot is created

    robot.pho_request_start_solution(252)
    robot.pho_request_ls_scan(1)
    robot.pho_ls_wait_for_scan()
    robot.pho_request_get_objects(1, 5)
    time.sleep(0.1)
    robot.pho_get_current_position()
    time.sleep(0.1)
    robot.pho_request_ls_get_vision_system_status(1)
    time.sleep(0.1)
    robot.pho_request_change_solution(253)
    time.sleep(0.1)
    robot.pho_request_ls_scan(1)
    robot.pho_ls_wait_for_scan()
    robot.pho_request_get_objects(1, 5)
    time.sleep(0.1)
    robot.pho_request_get_running_solution()
    time.sleep(0.1)
    #robot.pho_request_move_to_position()
    # time.sleep(0.2)
    # robot.pho_request_stop_solution()
    # time.sleep(2)
    robot.pho_request_get_available_solution()
    robot.close_connection()  #communication needs to be closed
    time.sleep(0.1)

def extract_object_coordinates(robot): # extract object coordinates [X,y,Z]
    try:
        # Replace 'objects' and 'coordinates' with actual attribute names from your response
        objects = robot.response_data.objects  # Example attribute; adjust accordingly
        
        if not objects:
            logging.info("No objects are found")
            return None

        # For simplicity, consider the first detected object
        first_object = objects[0]
        object_coords = first_object.coordinates  # Example attribute; adjust accordingly

        logging.info(f"Extracted Object Coordinates: {object_coords}")
        return object_coords

    except AttributeError:
        logging.error("Failed to extract object coordinates. Check the response data structure.")
        return None
    except Exception as e:
        logging.error(f"An error occurred while extracting object coordinates: {e}")
        return None

def format_coordinates(coords_mm):

    try:
        coords_m = [x / 1000.0 for x in coords_mm]
        return coords_m
    except TypeError:
        logging.error("Invalid type for coordinates. Expected list or tuple.")
        return None
    except Exception as e:
        logging.error(f"An error occurred while formatting coordinates: {e}")
        return None

def send_coordinates_to_robot(robot, coords): # function for sending coordinates to the robot
    
    try:
        # Replace 'pho_request_move_to_position' with the actual method name
        # and adjust parameters as required by your CommunicationLibrary
        robot.pho_request_move_to_position(coords[0], coords[1], coords[2])
        logging.info(f"Sent move command to position: {coords}")
    except AttributeError:
        logging.error("The method 'pho_request_move_to_position' does not exist in CommunicationLibrary.")
    except Exception as e:
        logging.error(f"An error occurred while sending move command: {e}")

def move_robot_to_position(robot, target_coords, tolerance=0.01, timeout=30): # function for moving robot to position

    try:
        start_time = time.time()
        while True:
            # Replace 'get_current_position' with the actual method to retrieve the robot's current position
            current_coords = robot.pho_get_current_position()
            distance = ((current_coords[0] - target_coords[0]) ** 2 +
                        (current_coords[1] - target_coords[1]) ** 2 +
                        (current_coords[2] - target_coords[2]) ** 2) ** 0.5
            if distance <= tolerance:
                logging.info(f"Robot reached target position: {current_coords}")
                break
            if time.time() - start_time > timeout:
                raise TimeoutError("Robot did not reach the target position in time.")
            time.sleep(0.5)
    except AttributeError:
        logging.error("The method 'get_current_position' does not exist in CommunicationLibrary.")
    except Exception as e:
        logging.error(f"An error occurred while moving the robot: {e}")

def test_ls(): # main function for calling every function.
    """
    Tests the LS (Laser Scan) functionality of the robot.
    Extracts object coordinates from the camera and sends them to the robotic controller to move the robot.
    """
    robot = CommunicationLibrary.RobotRequestResponseCommunication()  # Create robot communication object
    try:
        logging.info(f"Connecting to robot at {CONTROLLER_IP}:{PORT}")
        robot.connect_to_server(CONTROLLER_IP, PORT)  # Establish communication

        logging.info("Starting solution 252")
        robot.pho_request_start_solution(252)

        logging.info("Initiating LS scan")
        robot.pho_request_ls_scan(1)  # Start LS scan with parameter 1
        robot.pho_ls_wait_for_scan()    # Wait for the scan to complete

        logging.info("Requesting objects detected in the scan")
        robot.pho_request_get_objects(1, 5)  # Get objects detected (parameters may vary)
        time.sleep(0.1)  # Short delay to ensure response is received

        logging.info("Retrieving vision system status")
        robot.pho_request_ls_get_vision_system_status(1)
        time.sleep(0.1)

        logging.info("Changing solution to 253")
        robot.pho_request_change_solution(253)
        time.sleep(0.1)

        logging.info("Initiating another LS scan")
        robot.pho_request_ls_scan(1)
        robot.pho_ls_wait_for_scan()

        logging.info("Requesting objects detected in the second scan")
        robot.pho_request_get_objects(1, 5)
        time.sleep(0.1)

        logging.info("Retrieving running solution")
        robot.pho_request_get_running_solution()
        time.sleep(0.1)

        logging.info("Retrieving available solutions")
        robot.pho_request_get_available_solution()

        # Extract object coordinates from the response
        object_coords = extract_object_coordinates(robot)
        if object_coords:
            # Format coordinates (e.g., convert from mm to m)
            formatted_coords = format_coordinates(object_coords)
            if formatted_coords:
                logging.info(f"Formatted Object Coordinates (meters): {formatted_coords}")

                # Send coordinates to the robot to move
                send_coordinates_to_robot(robot, formatted_coords)

                # Optional: Command the robot to move to the coordinates and wait until it reaches
                # Uncomment the line below if you wish to perform this action
                # move_robot_to_position(robot, formatted_coords)
            else:
                logging.warning("Failed to format object coordinates.")
        else:
            logging.warning("No object coordinates found in the response.")

    except CommunicationLibrary.ConnectionError as ce:
        logging.error(f"Connection failed: {ce}")
    except CommunicationLibrary.CommandError as cmd_err:
        logging.error(f"Command failed: {cmd_err}")
    except TimeoutError as te:
        logging.error(f"Operation timed out: {te}")
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e}")
    finally:
        logging.info("Closing connection to the robot.")
        robot.close_connection()
        time.sleep(0.1)  # Short delay after closing the connection

def calibration_extrinsic():
    robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
    robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

    robot.pho_request_start_automatic_calibration(6,1)
    # Load the JSON data
    file_path = 'extrinsic_calib_points.json'
    json_data = load_json_file(file_path)

    # add 9 calibration point
    for point in json_data:
        translation_mm = point["translation"]
        quaternion = point["quaternion"]
        translation_m = [x * 1000 for x in translation_mm] # mm to m
        tool_pose = translation_m + quaternion

        robot.pho_request_add_calibration_point(tool_pose)
        time.sleep(2)

    robot.pho_request_save_automatic_calibration()
    time.sleep(2)
    robot.pho_request_stop_automatic_calibration()


def calibration_handeye():
    robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
    robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

    robot.pho_request_start_automatic_calibration(6, 2)
    # Load the JSON data
    file_path = 'handeye_calib_points.json'
    json_data = load_json_file(file_path)

    # add 9 calibration point
    for point in json_data:
        translation_mm = point["translation"]
        quaternion = point["quaternion"]
        translation_m = [x * 1000 for x in translation_mm]  # mm to m
        tool_pose = translation_m + quaternion

        robot.pho_request_add_calibration_point(tool_pose)
        time.sleep(2)


    #robot.pho_request_save_automatic_calibration()

    robot.pho_request_save_automatic_calibration()
    time.sleep(2)
    robot.pho_request_stop_automatic_calibration()


# Function to load JSON data from a file
def load_json_file(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data


if __name__ == '__main__':
    # calibration_handeye()
    calibration_extrinsic()
    test_ls()
    #test_bps()
    send_coordinates_to_robot()
    while True:
        test_ls()
        #test_bps()
        send_coordinates_to_robot()
