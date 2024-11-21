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

def main(robot_handler):
    program_handler = Program(robot_handler)
    robot_status = RobotStatus(robot_handler)
    iterator = loopCount()
    tool_objects = {}

    sio_object = register_sio_callbacks(program_handler)

    # Initializing local tools for Program 'Program_001'
    tool_objects['NoTool'] = robot_handler.gripper(
        gripper_name='STANDARD_GRIPPER', tool_name='NoTool'
    )
    # Setting tool 'NoTool' for Program 'Program_001'
    try:
        current_tool = 'NoTool'
        TOOL_PARAM_LENGTH = 16  # Number of parameters required by set_tool
        current_tool_params = [0] * TOOL_PARAM_LENGTH
        robot_handler.set_tool(
            tool_name=current_tool, tool_params=current_tool_params
        )
    except Exception as e:
        # Use a public method to send the error message or print it
        try:
            program_handler.send_gui_message(
                f'Error setting tool {current_tool}: {str(e)}', 'Error'
            )
        except AttributeError:
            print(f'Error setting tool {current_tool}: {str(e)}')
        raise e

    # Execute servo-controlled joint motion (MoveJ) with specified target joint angles
    target_joints = [
        [0.0, -1.57, 1.57, 0.0, 0.0, 0.0],  # Position 1
        [0.5, -1.2, 1.2, 0.5, 0.5, 0.5],    # Position 2
        [-0.5, -1.8, 1.8, -0.5, -0.5, -0.5] # Position 3
    ]
    execute_moveJ(robot_handler, program_handler, target_joints)

def execute_moveJ(
    robot_handler, program_handler, target_joints, speed=0.2, acceleration=0.1
):
    """
    Execute a joint motion command using MoveJ.

    Parameters:
    - robot_handler: Instance managing robot operations.
    - program_handler: Instance managing program commands.
    - target_joints: List of target joint positions for the robot.
    - speed: Speed of the joint movement (in rad/s).
    - acceleration: Acceleration of the joint movement (in rad/s^2).
    """
    current_cmd_id = 3  # Start with an ID greater than any fixed cmd_id
    for joint_target in target_joints:
        # Wait for the next step signal if using step-by-step control
        # block_until_next_step(robot_handler)  # Uncomment if step control is needed

        motion_data = {
            "speed": speed,
            "acceleration": acceleration,
            "joints": joint_target,  # Changed key from 'joint_positions' to 'joints'
            "continuous_execution": False  # Wait for motion to complete
        }
        cmd_id = current_cmd_id
        current_cmd_id += 1  # Increment ID for the next command

        # Updated command type
        program_handler.set_command(cmd.MOVE_J, **motion_data, cmd_id=cmd_id)
        program_handler.execute([cmd_id])

        # Wait until the robot reaches the target position
        while not robot_handler.is_motion_completed(cmd_id):
            sleep(0.01)

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
