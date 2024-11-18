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

    # Attempt to control a joint, assuming 'move_to_position' is the correct method
    try:
        robot_handler.move_to_position(joint_name='X', position=0)  # Initial position
    except Exception as e:
        program_handler._Program__PS.socket_object.send_gui_message(f'Error initializing joint control: {str(e)}', 'Error')
        raise e

    # Execute movement for the specified joint/servo
    target_positions = [45, 90]  # Example target positions
    execute_joint_movement(robot_handler, program_handler, target_positions)

def execute_joint_movement(robot_handler, program_handler, target_positions):
    global current_cmd_id
    for position in target_positions:
        try:
            robot_handler.move_to_position(joint_name='X', position=position)
            current_cmd_id += 1  # Simulate command execution flow
            sleep(0.1)  # Simulate command delay
        except Exception as e:
            program_handler._Program__PS.socket_object.send_gui_message(f'Error during joint movement: {str(e)}', 'Error')
            break  # Exit the loop if an error occurs

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
