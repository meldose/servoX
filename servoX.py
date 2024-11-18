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

    # Execute Cartesian-controlled motion (servoX) with specified target coordinates
    target_coordinates = [
        [400, 500, 600, 0, 0, 0],  # Coordinate set 1
        [500, 400, 700, 0, 0, 0],  # Coordinate set 2
        [600, 300, 800, 0, 0, 0],  # Coordinate set 3
    ]
    execute_servoX(robot_handler, program_handler, target_coordinates)

def execute_servoX(robot_handler, program_handler, target_coordinates, speed=50.0, acceleration=50.0):
    global current_cmd_id
    for coordinates in target_coordinates:
        motion_data = {
            "speed": speed,
            "acceleration": acceleration,
            "target_coordinates": coordinates,
            "continuous_execution": True
        }
        cmd_id = current_cmd_id
        current_cmd_id += 1  # Increment ID for the next command
        program_handler.set_command(cmd.Cartesian, **motion_data, cmd_id=cmd_id)  # Assume cmd.Cartesian is your defined command for servoX
        program_handler.execute([cmd_id])
        sleep(0.1)  # Small delay to simulate continuous motion, adjust as needed

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
        print('Exception:', str(e))
    finally:
        sys.exit("Program completed.")
