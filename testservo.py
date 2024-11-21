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
STEP_SLEEP_INTERVAL = 0

# Global variables
cur_step: int = 0
is_new_step: bool = False
current_cmd_id = 3

class ConveyorSystem:
    def __init__(self):
        # Initialize any necessary components here
        self.conveyor_speed = 1.0  # Default speed
        self.item_position = [0, 0]  # Default item position

    def initialize(self):
        # Implement any startup procedures necessary for the conveyor
        print("Conveyor initialized with speed:", self.conveyor_speed)

    def read_sensors(self):
        # This method should interface with actual conveyor sensors
        # For demonstration, it returns static values
        return (self.conveyor_speed, self.item_position)

    def control_conveyor(self, command):
        # Implement control commands like 'start', 'stop'
        if command == 'start':
            print("Conveyor started")
        elif command == 'stop':
            print("Conveyor stopped")
        else:
            print("Unknown command")

def main(robot_handler, conveyor):
    program_handler = Program(robot_handler)
    robot_status = RobotStatus(robot_handler)
    iterator = loopCount()
    tool_objects = {}

    sio_object = register_sio_callbacks(program_handler)

    # Initializing local tools for Program 'Program_001'
    tool_objects['NoTool'] = robot_handler.gripper(gripper_name='STANDARD_GRIPPER', tool_name='NoTool')
    robot_handler.set_tool(tool_name='NoTool', tool_params=[0]*16)

    # Initial setup to synchronize with the conveyor
    conveyor.initialize()

    while True:
        sensor_data = conveyor.read_sensors()
        update_robot_position_based_on_conveyor(sensor_data, robot_handler, program_handler)

def update_robot_position_based_on_conveyor(sensor_data, robot_handler, program_handler):
    conveyor_speed, item_position = sensor_data
    robot_speed = conveyor_speed * 1.1  # Adjust speed factor as necessary
    target_joints = calculate_target_joints(item_position, robot_speed)
    execute_servoJ(robot_handler, program_handler, target_joints, speed=robot_speed, acceleration=50.0)

def calculate_target_joints(item_position, robot_speed):
    # Placeholder function to calculate target joints based on item position
    # This should be replaced with actual calculation logic
    return [
        [1.5708, -1.5708, 0.7854, -0.7854, 1.5708, 0],
        [0, 1.5708, -1.5708, 0.7854, -0.7854, 1.5708],
    ]

def execute_servoJ(robot_handler, program_handler, target_joints, speed=50.0, acceleration=50.0):
    global current_cmd_id
    for joint_target in target_joints:
        cmd_id = current_cmd_id
        current_cmd_id += 1
        program_handler.set_command(cmd.Joint, target_joint=joint_target, speed=speed, acceleration=acceleration, cmd_id=cmd_id)
        program_handler.execute([cmd_id])
        sleep(0.1)

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

def block_until_next_step():
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
    conveyor = ConveyorSystem()  # This now references the defined class
    try:
        main(robot_handler, conveyor)
    except Exception as e:
        print('Exception:', str(e))
    finally:
        sys.exit("Program completed.")
