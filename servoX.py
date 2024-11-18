from time import sleep
import signal
import sys

from neurapy.state_flag import cmd
from neurapy.component import Component
from neura_apps.gui_program.program import Program
from neurapy.robot import Robot
from neurapy.commands.state.robot_status import RobotStatus
from neurapy.loop_counter import loopCount
from neurapy.utils import CmdIDManager
from neurapy.socket_client import get_sio_client_singleton_instance

# Global constants
STEP_SLEEP_INTERVAL = 0.01

# Global variables
cur_step: int = 0
is_new_step: bool = False
current_cmd_id = 3  # Start with an ID greater than any fixed cmd_id

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
    except Exception as e:
        program_handler._Program__PS.socket_object.send_gui_message(f'Error setting tool {current_tool}: {str(e)}', 'Error')
        raise e

    # Test basic Cartesian command
    test_cartesian_command(robot_handler, program_handler)

def test_cartesian_command(robot_handler, program_handler):
    cmd_id = 99  # Use a distinct ID for testing
    motion_data = {
        "speed": 50,
        "acceleration": 50,
        "target_coordinates": [100, 200, 300, 0, 0, 0],
        "continuous_execution": False
    }
    print(f"Preparing to execute test Cartesian command with data: {motion_data}")

    # Validate coordinates and parameters
    if not validate_motion_parameters(motion_data):
        print("Validation failed for motion parameters.")
        return

    try:
        program_handler.set_command(cmd.Cartesian, **motion_data, cmd_id=cmd_id)
        print(f"Command set successfully, proceeding to execute with ID {cmd_id}")
        program_handler.execute([cmd_id])
        print("Test Cartesian command executed successfully.")
    except Exception as e:
        print(f"Failed to execute test Cartesian command with ID {cmd_id}: {str(e)}")
        raise

def validate_motion_parameters(motion_data):
    x, y, z, rx, ry, rz = motion_data['target_coordinates']
    max_speed = 100  # Define maximum speed based on your robot's specifications
    max_acceleration = 100  # Define maximum acceleration

    if not (0 <= motion_data['speed'] <= max_speed):
        print(f"Invalid speed: {motion_data['speed']}")
        return False
    if not (0 <= motion_data['acceleration'] <= max_acceleration):
        print(f"Invalid acceleration: {motion_data['acceleration']}")
        return False
    if not (-1000 <= x <= 1000 and -1000 <= y <= 1000 and -1000 <= z <= 1000):  # Example bounds
        print(f"Invalid target coordinates: {x}, {y}, {z}")
        return False
    return True

def register_sio_callbacks(program_handler):
    sio_handler = get_sio_client_singleton_instance()
    sio_register = sio_handler.get_sio_client_register_obj()
    sio_register.on("StepByStep", handle_sio_step_by_step)
    return sio_handler.get_sio_client_obj()

def handle_sio_step_by_step(data):
    global cur_step
    global is_new_step
    cur_step = int(data.get('id', -1))
    is_new_step = True

def block_until_next_step(robot):
    global is_new_step
    while not is_new_step:
        sleep(STEP_SLEEP_INTERVAL)
    is_new_step = False

def signal_handler(signum, frame):
    sys.exit("Received termination signal, exiting.")

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    robot_handler = Robot()
    try:
        main(robot_handler)
    except Exception as e:
        print(f"Exception during robot operation: {str(e)}")
    finally:
        sys.exit("Program completed.")
