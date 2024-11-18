# Integrated Robot Control and Servo Tracking Program

import time
import logging
import signal
import sys
import argparse
import threading
from time import sleep
import os

from neurapy.state_flag import cmd
from neurapy.component import Component
from neura_apps.gui_program.program import Program
from neurapy.robot import Robot
from neurapy.commands.state.robot_status import RobotStatus
from neurapy.loop_counter import loopCount
from neurapy.utils import CmdIDManager
from neurapy.socket_client import get_sio_client_singleton_instance

# ============================
# Configuration and Setup
# ============================

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,  # Set to DEBUG for detailed output
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        # You can add a FileHandler here to log to a file if needed
    ]
)
logger = logging.getLogger(__name__)

# Global flags for graceful shutdown
running = True

# Global constants
STEP_SLEEP_INTERVAL = 0.01

# # Global variables for step management
# cur_step: int = 0
# is_new_step: bool = False

# Lock for thread-safe operations on global variables
step_lock = threading.Lock()

# Placeholder for Components to allow access in signal_handler
hr = None
rts = None

def signal_handler(sig, frame):
    global running
    logger.info('Interrupt received, stopping all operations...')
    running = False
    try:
        hr.callService('ResetPythonProgramStatus', [])
        logger.info("Called service: ResetPythonProgramStatus")
    except Exception as e:
        logger.error(f"Failed to call ResetPythonProgramStatus: {e}")
    try:
        rts.callService('StopPythonScript', [])
        logger.info("Called service: StopPythonScript")
    except Exception as e:
        logger.error(f"Failed to call StopPythonScript: {e}")
    sys.exit(0)

# Register the signal handler for graceful shutdown
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# ============================
# Argument Parsing
# ============================

def parse_arguments():
    parser = argparse.ArgumentParser(description='Integrated Robot Control and Servo Tracking Program')
    parser.add_argument('--conveyor_speed', type=float, default=0.2,
                        help='Conveyor speed in meters per second (default: 0.2)')
    parser.add_argument('--time_interval', type=float, default=0.1,
                        help='Time interval in seconds for each update (default: 0.1)')
    parser.add_argument('--tracking_duration', type=int, default=10,
                        help='Total time to track the object in seconds (default: 10)')
    parser.add_argument('--config_path', type=str, required=True,
                        help='Path to robot configuration file (required)')
    parser.add_argument('--acceleration', type=float, default=1.2,
                        help='Acceleration for servo movement (default: 1.2)')
    parser.add_argument('--velocity', type=float, default=0.25,
                        help='Velocity for servo movement (default: 0.25)')
    return parser.parse_args()

# ============================
# Socket.IO Callback Handling
# ============================

def register_sio_callbacks(program_handler: Program):
    sio_handler = get_sio_client_singleton_instance()
    sio_register = sio_handler.get_sio_client_register_obj()
    sio_register.on("StepByStep", handle_sio_step_by_step)
    sio_obj_client = sio_handler.get_sio_client_obj()
    return sio_obj_client

def handle_sio_step_by_step(data: dict):
    global cur_step
    global is_new_step
    logger.debug(f"Handling socket message on channel 'StepByStep' with data {data}")
    try:
        new_step = int(data.get('id', -1))
        with step_lock:
            if new_step != cur_step:
                cur_step = new_step
                is_new_step = True
                logger.info(f"Step updated to {cur_step}")
    except (ValueError, TypeError) as ex:
        logger.error(f"Invalid step data received: {data}. Error: {ex}")

def read_cur_step_from_gui() -> int:
    global cur_step
    with step_lock:
        return cur_step

def block_until_next_step(robot: Robot):
    global cur_step
    global is_new_step
    old_step = read_cur_step_from_gui()
    while running and robot.is_robot_in_semi_automatic_mode() and read_cur_step_from_gui() <= old_step and not is_new_step:
        sleep(STEP_SLEEP_INTERVAL)
    with step_lock:
        is_new_step = False

# ============================
# Servo Tracking Function
# ============================

def servo_tracking_with_conveyor(robot, conveyor_speed, time_interval, tracking_duration, acceleration, velocity):
    start_time = time.time()
    logger.info("Starting servo tracking with conveyor.")
    while running and (time.time() - start_time) < tracking_duration:
        try:
            current_position = robot.get_tcp_position()
            if current_position is None:
                logger.warning("Current position is None. Check robot status.")
                break

            # Ensure the position is a mutable list
            new_position = list(current_position)
            new_position[0] += conveyor_speed * time_interval  # Update X position

            # Optional: Validate new_position within robot's workspace
            if hasattr(robot, 'is_position_within_workspace'):
                if not robot.is_position_within_workspace(new_position):
                    logger.warning(f"New position {new_position} is out of workspace bounds.")
                    break

            # Execute servo movement with acceleration and velocity
            robot.servox(new_position, a=acceleration, v=velocity)
            logger.debug(f"Moved to position: {new_position}")

            time.sleep(time_interval)
        except AttributeError as ae:
            logger.error(f"Attribute Error: {ae}")
            break
        except Exception as e:
            logger.exception("Error during servo tracking")
            break

    logger.info("Servo tracking completed.")

# ============================
# Main Program Function
# ============================

def main_program(args, robot_handler):
    # --------------------------  CREATING Robot & Program & RobotStatus OBJECTS  -----------------------

    program_handler = Program(robot_handler)
    robot_status = RobotStatus(robot_handler)
    iterator = loopCount()
    tool_objects = {}

    sio_object = register_sio_callbacks(program_handler)

    # Set initial value of dynamic id higher than the highest fixed cmd_id, so it does not intervene
    id_manager = CmdIDManager(3)

    # Initializing local tools for Program 'Program_001'
    tool_objects['NoTool'] = robot_handler.gripper(gripper_name='STANDARD_GRIPPER', tool_name='NoTool')
    # Setting tool 'NoTool' for Program 'Program_001'
    try:
        current_tool = 'NoTool'
        current_tool_params = [0] * 16
        robot_handler.set_tool(tool_name=current_tool, tool_params=current_tool_params)
        logger.info(f"Tool '{current_tool}' set with parameters {current_tool_params}")
    except Exception as e:
        program_handler._Program__PS.socket_object.send_gui_message(
            f'Error {current_tool} tool not set properly: {str(e)}', 'Error'
        )
        logger.error(f"Error setting tool '{current_tool}': {e}")
        raise e

    # -------------------------------  INITIAL JOINT STATE & CARTESIAN POSE  ----------------------------

    previous_joint_angles = robot_status.getRobotStatus('jointAngles')
    previous_cartesian_poses = robot_handler.ik_fk(
        "fk", target_angle=previous_joint_angles, tool_params=current_tool_params
    )

    motion_data_initial = {
        "speed": 50.0,
        "acceleration": 50.0,
        "target_joint": [
            previous_joint_angles,
            previous_joint_angles
        ]
    }
    # Planning mj to first point | static planning
    program_handler.set_command(
        cmd.Joint,
        **motion_data_initial,
        cmd_id=1,
        current_joint_angles=previous_joint_angles,
        reusable_id=0
    )
    program_handler.execute([1])
    logger.info("Executed initial joint movement command (cmd_id=1).")

    # --------------------------------------  PLANNING COMMANDS  ----------------------------------------

    # Planning statements for mj_001 (MoveJoint)
    motion_data_mj_001 = {
        "speed": 50.0,
        "acceleration": 50.0,
        "enable_blending": False,
        "target_joint": [
            previous_joint_angles,
            [
                1.5708001839725434,
                0,
                -2.49582083,
                0,
                0,
                0
            ],
            [
                -1.3719472167217317,
                0.3183455847773389,
                1.2019206068329313,
                2.644990459600516,
                1.2970963723092435,
                2.849789749302829
            ]
        ],
        "target_pose": [
            previous_cartesian_poses,
            [
                0,
                -0.347247,
                0.139187,
                3.1416,
                -0.6458,
                -1.5708
            ],
            [
                0.18400071652285083,
                -0.5485334059162619,
                0.7369915126699076,
                -0.5357636754038769,
                0.0438629109613539,
                -1.758100481730435
            ]
        ],
        "control_mode": 0.0,
        "force_vector": [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ]
    }
    # Planning mj_001 | static planning
    program_handler.set_command(
        cmd.Joint,
        **motion_data_mj_001,
        cmd_id=2,
        current_joint_angles=previous_joint_angles,
        reusable_id=1
    )
    logger.info("Planned MoveJoint command (cmd_id=2).")

    # --------------------------------------  EXECUTING COMMANDS  ---------------------------------------

    # Executing mj_001 | static planning | no previously generated motion
    block_until_next_step(robot_handler)
    program_handler.execute([2])
    logger.info("Executed MoveJoint command (cmd_id=2).")
    sio_object.send_gui_message({"state": "FINISHED"}, socket_name="StepByStep")
    logger.info("Sent FINISHED state via Socket.IO.")

# ============================
# Robot Initialization
# ============================

def initialize_robot(config_path):
    if not os.path.isfile(config_path):
        logger.error(f"Configuration file not found at: {config_path}")
        sys.exit(1)
    try:
        robot = Robot(config_path=config_path)
        logger.info(f"Robot initialized with config: {config_path}")
        # Optionally, verify connection here if Robot class supports it
        if hasattr(robot, 'is_connected'):
            if not robot.is_connected():
                logger.error("Robot is not connected. Please check the connection.")
                sys.exit(1)
            else:
                logger.info("Robot connection verified.")
        return robot
    except FileNotFoundError:
        logger.error(f"Configuration file not found at: {config_path}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Failed to initialize Robot: {e}")
        sys.exit(1)

# ============================
# Main Function
# ============================

def main():
    global robot_handler, program_handler, rts, hr  # To allow signal_handler access

    args = parse_arguments()

    logger.info("Starting Integrated Robot Control and Servo Tracking Program")
    logger.info(f"Parameters: Conveyor Speed={args.conveyor_speed} m/s, "
                f"Time Interval={args.time_interval} s, "
                f"Tracking Duration={args.tracking_duration} s, "
                f"Acceleration={args.acceleration}, "
                f"Velocity={args.velocity}, "
                f"Config Path={args.config_path}")

    # Initialize Robot
    robot_handler = initialize_robot(args.config_path)

    # Initialize Components
    program_handler = None  # Will be initialized in main_program
    rts = Component(robot_handler, "RTS")
    hr = Component(robot_handler, "HR")

    # Start main_program in a separate thread
    main_program_thread = threading.Thread(target=main_program, args=(args, robot_handler))
    main_program_thread.start()

    # Start servo tracking in a separate thread
    servo_tracking_thread = threading.Thread(
        target=servo_tracking_with_conveyor,
        args=(
            robot_handler,
            args.conveyor_speed,
            args.time_interval,
            args.tracking_duration,
            args.acceleration,
            args.velocity
        )
    )
    servo_tracking_thread.start()

    # Wait for both threads to complete
    main_program_thread.join()
    servo_tracking_thread.join()

    # Final shutdown procedures
    try:
        # Setting tool parameter into robot handler
        robot_handler.set_tool(
            tool_name='NoTool',
            tool_params=[0] * 16
        )
        logger.info("Tool 'NoTool' reset to default parameters.")
    except Exception as e:
        logger.error(f"Error resetting tool parameters: {e}")

    try:
        # Optionally, finish the program handler if necessary
        if program_handler:
            program_handler.finish()
            logger.info("Program handler finished.")
    except Exception as e:
        logger.error(f"Error finishing program handler: {e}")

    try:
        # Stop Python script via RTS component
        rts.callService('StopPythonScript', [])
        logger.info("Stopped Python script via RTS component.")
    except Exception as e:
        logger.error(f"Error stopping Python script: {e}")

    logger.info("Integrated program has terminated gracefully.")

# ============================
# Entry Point
# ============================

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logger.exception(f"Unhandled exception: {e}")
        # Attempt to perform cleanup
        try:
            hr.callService('ResetPythonProgramStatus', [])
        except:
            logger.error("Failed to call ResetPythonProgramStatus during exception handling.")
        try:
            rts.callService('StopPythonScript', [])
        except:
            logger.error("Failed to call StopPythonScript during exception handling.")
        sys.exit(1)
