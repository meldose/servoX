import sys
import signal
from time import sleep
from neurapy.state_flag import cmd
from neurapy.component import Component
from neura_apps.gui_program.program import Program
from neurapy.robot import Robot
from neurapy.commands.state.robot_status import RobotStatus
from neurapy.loop_counter import loopCount
from neurapy.utils import CmdIDManager
from neurapy.socket_client import get_sio_client_singleton_instance
import logging

# Setup basic logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# Global constants
STEP_SLEEP_INTERVAL = 0.01
ENABLE_STEP_BY_STEP = False  # Set to False to disable socket dependency for testing

# Global variables
cur_step: int = 0
is_new_step: bool = False
current_cmd_id = 3  # Start with an ID greater than any fixed cmd_id

def is_robot_ready(robot_status):
    """
    Check if the robot is in a ready state.
    """
    try:
        status = robot_status.get_state()  # Replace with the correct method if available
        logging.info("Robot status: %s", status)
        if status in ["READY", "OPERATIONAL"]:
            return True
        else:
            logging.warning("Robot not ready. Current state: %s", status)
            return False
    except AttributeError as e:
        logging.error("RobotStatus object has no attribute 'get_state': %s", e)
        return False
    except Exception as e:
        logging.error("Error checking robot readiness: %s", e)
        return False

def validate_motion_parameters(motion_data):
    x, y, z, rx, ry, rz = motion_data['target_coordinates']
    max_speed = 100  # Define maximum speed based on your robot's specifications
    max_acceleration = 100  # Define maximum acceleration

    if not (0 <= motion_data['speed'] <= max_speed):
        logging.error("Invalid speed: %d", motion_data['speed'])
        return False
    if not (0 <= motion_data['acceleration'] <= max_acceleration):
        logging.error("Invalid acceleration: %d", motion_data['acceleration'])
        return False
    if not (-1000 <= x <= 1000 and -1000 <= y <= 1000 and -1000 <= z <= 1000):  # Example bounds
        logging.error("Invalid target coordinates: %d, %d, %d", x, y, z)
        return False
    return True

def test_cartesian_command(robot_handler, program_handler, robot_status):
    cmd_id = 99  # Use a distinct ID for testing
    motion_data = {
        "speed": 10,  # Reduced speed for safety
        "acceleration": 10,  # Reduced acceleration
        "target_coordinates": [50, 50, 50, 0, 0, 0],  # Simplified movement
        "continuous_execution": False
    }
    logging.info("Preparing to execute simplified Cartesian command with data: %s", motion_data)

    if not is_robot_ready(robot_status):
        logging.error("Robot is not in a ready state. Please check the robot's status.")
        return False

    if not validate_motion_parameters(motion_data):
        logging.error("Validation failed for motion parameters. Data: %s", motion_data)
        return False

    try:
        logging.debug("Setting Cartesian command with ID %d.", cmd_id)
        program_handler.set_command(cmd.Cartesian, **motion_data, cmd_id=cmd_id)
        logging.debug("Executing Cartesian command with ID %d.", cmd_id)
        program_handler.execute([cmd_id])
        logging.info("Simplified Cartesian command executed successfully.")
        return True
    except Exception as e:
        logging.error("Failed to execute simplified Cartesian command with ID %d: %s", cmd_id, e)
        return False

def register_sio_callbacks(program_handler):
    if ENABLE_STEP_BY_STEP:
        sio_handler = get_sio_client_singleton_instance()
        sio_register = sio_handler.get_sio_client_register_obj()
        sio_register.on("StepByStep", handle_sio_step_by_step)
        return sio_handler.get_sio_client_obj()
    else:
        logging.info("Step-by-step mode disabled.")
        return None

def handle_sio_step_by_step(data):
    global cur_step
    global is_new_step
    cur_step = int(data.get('id', -1))
    is_new_step = True

def block_until_next_step(robot):
    if not ENABLE_STEP_BY_STEP:
        return  # Skip blocking if step-by-step mode is disabled
    global is_new_step
    while not is_new_step:
        sleep(STEP_SLEEP_INTERVAL)
    is_new_step = False

def signal_handler(signum, frame):
    logging.info("Received termination signal, exiting.")
    sys.exit()

def main(robot_handler):
    program_handler = Program(robot_handler)
    robot_status = RobotStatus(robot_handler)
    iterator = loopCount()
    tool_objects = {}

    sio_object = register_sio_callbacks(program_handler)

    tool_objects['NoTool'] = robot_handler.gripper(gripper_name='STANDARD_GRIPPER', tool_name='NoTool')
    try:
        current_tool = 'NoTool'
        current_tool_params = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        robot_handler.set_tool(tool_name=current_tool, tool_params=current_tool_params)
        logging.debug("Tool %s set with parameters: %s", current_tool, current_tool_params)
    except Exception as e:
        logging.error("Error setting tool %s: %s", current_tool, e)
        raise e

    # Test basic Cartesian command
    if test_cartesian_command(robot_handler, program_handler, robot_status):
        logging.info("Cartesian test command successful, robot should now be moving.")
    else:
        logging.error("Cartesian test command failed. Please check robot readiness, parameters, or connection.")

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    robot_handler = Robot()
    try:
        main(robot_handler)
    except Exception as e:
        logging.error("Exception during robot operation: %s: %s", type(e).__name__, e)
    finally:
        sys.exit("Program completed.")
