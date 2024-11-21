
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

    # Initializing local tools for Program 'Program_001'
    tool_objects['NoTool'] = robot_handler.gripper(gripper_name='STANDARD_GRIPPER', tool_name='NoTool')
    # Setting tool 'NoTool' for Program 'Program_001'
    try:
        current_tool = 'NoTool'
        current_tool_params = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        robot_handler.set_tool(tool_name=current_tool, tool_params=current_tool_params)
    except Exception as e:
        program_handler._Program__PS.socket_object.send_gui_message(f'Error setting tool {current_tool}: {str(e)}', 'Error')
        raise e

    # Execute servo-controlled joint motion (servoJ) with specified target positions
    target_joints = [
        [1.5708, -1.5708, 0.7854, -0.7854, 1.5708, 0],  # Position 1
        [0, 1.5708, -1.5708, 0.7854, -0.7854, 1.5708],  # Position 2
        [1.0, -0.5, 0.5, 0, 1.0, 0.5],  # Position 3
        [1.2, -0.7, 0.7, 0, 1.2, 0.8], # Position 4
        [1.0, -0.5, 0.5, 0, 1.0, 0],  # Position 5
    ]
    execute_servoJ(robot_handler, program_handler, target_joints)

def execute_servoJ(robot_handler, program_handler, target_joints, speed=50.0, acceleration=50.0):
    global current_cmd_id
    """
    Execute a servo-controlled joint motion command.

    Parameters:
    - robot_handler: Instance managing robot operations.
    - program_handler: Instance managing program commands.
    - target_joints: List of target joint angles for the robot.
    - speed: Speed of the joint movement.
    - acceleration: Acceleration of the joint movement.
    """
    for joint_target in target_joints:
        motion_data = {
            "speed": speed,
            "acceleration": acceleration,
            "target_joint": joint_target,
            "continuous_execution": True
        }
        cmd_id = current_cmd_id
        current_cmd_id += 1  # Increment ID for the next command
        program_handler.set_command(cmd.Joint, **motion_data, cmd_id=cmd_id)
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
