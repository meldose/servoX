c
import sys
import signal
import threading
import random
from time import sleep
import logging

from gpiozero import Servo  # Import gpiozero library

from neurapy.state_flag import cmd
from neurapy.component import Component
from neura_apps.gui_program.program import Program
from neurapy.robot import Robot
from neurapy.commands.state.robot_status import RobotStatus
from neurapy.loop_counter import loopCount
from neurapy.utils import CmdIDManager
from neurapy.socket_client import get_sio_client_singleton_instance

# ---------------------------- Logging Configuration ----------------------------
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler("robot_control.log"),
        logging.StreamHandler(sys.stdout)
    ]
)

# ----------------------- Global Constants -----------------------
STEP_SLEEP_INTERVAL = 0.01

# ----------------------- Global Variables for Step Management -----------------------
cur_step = 0
is_new_step = False
step_lock = threading.Lock()

# ----------------------- Servo Control Integration -----------------------

# Define GPIO pins for servos
SERVO_J_PIN = 17  # GPIO17 for Servo J
SERVO_X_PIN = 27  # GPIO27 for Servo X

# Initialize servos
servo_j = Servo(SERVO_J_PIN)  # Initialize Servo J
servo_x = Servo(SERVO_X_PIN)  # Initialize Servo X


def set_servo_position(servo, position):
    """
    Set the servo position.

    :param servo: Servo object
    :param position: Float between -1 (full left) and +1 (full right)
    """
    try:
        servo.value = position
        logging.info(f"Setting Servo on GPIO{servo.pin.number} to position {position}")
    except Exception as e:
        logging.error(f"Failed to set servo position on GPIO{servo.pin.number}: {e}")


def set_servo_speed(servo, speed):
    """
    Simulate setting the servo speed by adjusting its position.

    :param servo: Servo object
    :param speed: Float value representing speed (simulated)
    """
    try:
        servo.value = speed
        logging.info(f"Setting Servo on GPIO{servo.pin.number} to speed {speed}")
    except Exception as e:
        logging.error(f"Failed to set servo speed on GPIO{servo.pin.number}: {e}")


def get_object_position():
    """
    Placeholder function to get the object's position from sensors.
    Implement sensor reading logic here.

    :return: Float representing object position
    """
    # Example: Return a random position value
    # Replace with actual sensor input
    position = random.uniform(-1, 1)
    logging.info(f"Retrieved object position: {position}")
    return position


def calculate_servo_j_position(position):
    """
    Calculate the servo J position based on object position.

    :param position: Float representing object position
    :return: Float representing servo J position
    """
    # Example: Direct mapping
    logging.info(f"Calculated Servo J position: {position} based on object position: {position}")
    return position


def calculate_servo_x_speed():
    """
    Calculate the servo X speed based on tracking requirements.

    :return: Float representing servo X speed
    """
    # Example: Constant speed
    speed = 0.5
    logging.info(f"Calculated Servo X speed: {speed}")
    return speed


def conveyor_tracking_system(stop_event):
    """
    Main function to control the conveyor tracking system.
    Adjusts servo positions based on sensor input or predefined logic.
    Runs until the stop_event is set.

    :param stop_event: threading.Event to signal thread termination
    """
    logging.info("Conveyor tracking system started.")
    try:
        while not stop_event.is_set():
            # Move Servo J to track an object position
            object_position = get_object_position()
            servo_j_position = calculate_servo_j_position(object_position)
            set_servo_position(servo_j, servo_j_position)

            # Adjust Servo X speed based on tracking needs
            desired_speed = calculate_servo_x_speed()
            set_servo_speed(servo_x, desired_speed)

            sleep(0.1)  # Adjust the loop delay as needed
    except Exception as e:
        logging.error(f"Conveyor tracking system encountered an error: {e}")
    finally:
        logging.info("Conveyor tracking system stopped.")
        servo_j.value = None
        servo_x.value = None


# ----------------------- End of Servo Control Integration -----------------------


def main(robot_handler, stop_event):
    """
    Main function to initialize and execute robot commands.

    :param robot_handler: Robot object
    :param stop_event: threading.Event to signal thread termination
    """
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
        current_tool_params = [0] * 16  # List of 16 zeros
        robot_handler.set_tool(tool_name=current_tool, tool_params=current_tool_params)
        logging.info(f"Tool '{current_tool}' set with parameters {current_tool_params}.")
    except Exception as e:
        error_message = f"Error {current_tool} tool not set properly: {str(e)}"
        logging.error(error_message)
        program_handler._Program__PS.socket_object.send_gui_message(error_message, 'Error')
        raise e

    # -------------------------------  INITIAL JOINT STATE & CARTESIAN POSE  ----------------------------

    try:
        previous_joint_angles = robot_status.getRobotStatus('jointAngles')
        previous_cartesian_poses = robot_handler.ik_fk("fk", target_angle=previous_joint_angles, tool_params=current_tool_params)

        motion_data = {
            "speed": 50.0,
            "acceleration": 50.0,
            "target_joint": [
                previous_joint_angles,
                previous_joint_angles
            ]
        }
        # Planning and executing the first joint movement
        program_handler.set_command(
            cmd.Joint,
            **motion_data,
            cmd_id=1,
            current_joint_angles=previous_joint_angles,
            reusable_id=0
        )
        program_handler.execute([1])
        logging.info("Executed first joint movement command.")
    except Exception as e:
        logging.error(f"Error during initial joint state setup: {e}")
        raise e

    # --------------------------------------  PLANNING COMMANDS  ----------------------------------------

    try:
        # Planning second joint movement with specific target angles and poses
        motion_data = {
            "speed": 50.0,
            "acceleration": 50.0,
            "enable_blending": False,
            "target_joint": [
                previous_joint_angles,
                [1.5708001839725434, 0, -2.49582083, 0, 0, 0],
                [-1.3719472167217317, 0.3183455847773389, 1.2019206068329313, 2.644990459600516, 1.2970963723092435, 2.849789749302829]
            ],
            "target_pose": [
                previous_cartesian_poses,
                [0, -0.347247, 0.139187, 3.1416, -0.6458, -1.5708],
                [0.18400071652285083, -0.5485334059162619, 0.7369915126699076, -0.5357636754038769, 0.0438629109613539, -1.758100481730435]
            ],
            "control_mode": 0.0,
            "force_vector": [0.0] * 6
        }
        program_handler.set_command(
            cmd.Joint,
            **motion_data,
            cmd_id=2,
            current_joint_angles=previous_joint_angles,
            reusable_id=1
        )
        logging.info("Planned second joint movement command.")
    except Exception as e:
        logging.error(f"Error during planning of second joint movement: {e}")
        raise e

    # --------------------------------------  EXECUTING COMMANDS  ---------------------------------------

    try:
        # Executing the second joint movement
        block_until_next_step(robot_handler)
        program_handler.execute([2])
        logging.info("Executed second joint movement command.")
        sio_object.send_gui_message({"state": "FINISHED"}, socket_name="StepByStep")
        logging.info("Sent FINISHED state to GUI.")
    except Exception as e:
        logging.error(f"Error during execution of commands: {e}")
        raise e

    # Optionally, you can add more robot commands or steps here


def register_sio_callbacks(program_handler):
    """
    Register Socket.IO callbacks.

    :param program_handler: Program object
    :return: Socket.IO client object
    """
    sio_handler = get_sio_client_singleton_instance()
    sio_register = sio_handler.get_sio_client_register_obj()

    @sio_register.on("StepByStep")
    def on_step_by_step(data):
        if isinstance(data, dict) and 'id' in data:
            handle_sio_step_by_step(data)
        else:
            logging.warning(f"Received invalid data on StepByStep: {data}")

    sio_obj_client = sio_handler.get_sio_client_obj()
    logging.info("Registered Socket.IO callbacks.")
    return sio_obj_client


def handle_sio_step_by_step(data):
    """
    Handle incoming Socket.IO messages for step-by-step execution.

    :param data: Dictionary containing step data
    """
    global cur_step
    global is_new_step
    logging.info(f"Handling Socket.IO message on channel 'StepByStep' with data: {data}")
    try:
        new_step = int(data.get('id', -1))
        with step_lock:
            old_step = cur_step
            cur_step = new_step
            if old_step != cur_step:
                is_new_step = True
                logging.info(f"Updated step from {old_step} to {cur_step}")
    except ValueError:
        logging.error(f"Invalid step ID received: {data.get('id')}")


def read_cur_step_from_gui():
    """
    Read the current step from the GUI.

    :return: Integer representing the current step
    """
    global cur_step
    with step_lock:
        return cur_step


def block_until_next_step(robot):
    """
    Block execution until the next step is received from the GUI.

    :param robot: Robot object
    """
    global cur_step
    global is_new_step
    logging.info("Blocking until the next step is received from the GUI.")
    while robot.is_robot_in_semi_automatic_mode():
        with step_lock:
            current_step = cur_step
            new_step = is_new_step
        if new_step and cur_step > current_step:
            with step_lock:
                is_new_step = False
            logging.info(f"Proceeding to step {cur_step}.")
            break
        sleep(STEP_SLEEP_INTERVAL)


def cleanup(stop_event, servo_thread, robot_handler, rts):
    """
    Perform cleanup operations before shutting down.

    :param stop_event: threading.Event to signal thread termination
    :param servo_thread: Thread object for servo control
    :param robot_handler: Robot object
    :param rts: Component object for RTS
    """
    logging.info("Performing cleanup...")
    # Stop the servo control thread
    try:
        stop_event.set()
        servo_thread.join(timeout=1)
        logging.info("Servo control thread stopped.")
    except Exception as e:
        logging.error(f"Error stopping servo thread: {e}")

    # Reset tool parameters
    try:
        robot_handler.set_tool(tool_name='NoTool', tool_params=[0] * 16)
        logging.info("Tool parameters reset to 'NoTool'.")
    except Exception as e:
        logging.error(f"Error resetting tool parameters: {e}")

    # Stop Python script
    try:
        rts.callService('StopPythonScript', [])
        logging.info("Python script stopped via RTS.")
    except Exception as e:
        logging.error(f"Error stopping Python script: {e}")


def signal_handler(signum, frame, stop_event, servo_thread, robot_handler, rts):
    """
    Handles termination signals to gracefully shut down the program.

    :param signum: Signal number
    :param frame: Current stack frame
    :param stop_event: threading.Event to signal thread termination
    :param servo_thread: Thread object for servo control
    :param robot_handler: Robot object
    :param rts: Component object for RTS
    """
    logging.info(f"Received signal {signum}, initiating graceful shutdown.")
    try:
        # Reset robot program status
        hr = Component(robot_handler, "HR")
        hr.callService('ResetPythonProgramStatus', [])
        logging.info("Robot program status reset.")
    except Exception as e:
        logging.error(f"Error resetting Python program status: {e}")

    # Perform cleanup
    cleanup(stop_event, servo_thread, robot_handler, rts)

    # Finally, exit the script
    logging.info("Shutdown complete. Exiting.")
    sys.exit()


if __name__ == "__main__":
    # Initialize robot handler and program handler
    robot_handler = Robot()
    program_handler = Program(robot_handler)
    rts = Component(robot_handler, "RTS")

    # Create an event to signal the servo thread to stop
    stop_event = threading.Event()

    # Initialize and start the conveyor tracking system in a separate thread
    servo_thread = threading.Thread(target=conveyor_tracking_system, args=(stop_event,), daemon=True)
    servo_thread.start()
    logging.info("Started conveyor tracking system thread.")

    # Register signal handlers with additional arguments using lambda
    # Since signal handlers can't accept additional arguments directly
    signal.signal(signal.SIGINT, lambda s, f: signal_handler(s, f, stop_event, servo_thread, robot_handler, rts))
    signal.signal(signal.SIGTERM, lambda s, f: signal_handler(s, f, stop_event, servo_thread, robot_handler, rts))
    logging.info("Registered signal handlers for SIGINT and SIGTERM.")

    try:
        main(robot_handler, stop_event)
    except Exception as e:
        error_message = f"Exception occurred: {str(e)}"
        logging.error(error_message)
        try:
            hr = Component(robot_handler, "HR")
            hr.callService('ResetPythonProgramStatus', [])
            logging.info("Robot program status reset after exception.")
        except Exception as reset_e:
            logging.error(f"Error resetting Python program status: {reset_e}")
        try:
            program_handler._Program__PS.socket_object.send_gui_message(
                'Error: ' + str(e)
            )
            logging.info("Error message sent to GUI.")
        except Exception as socket_e:
            logging.error(f"Error sending GUI message: {socket_e}")
    finally:
        # Ensure that the servo thread is stopped
        cleanup(stop_event, servo_thread, robot_handler, rts)
        logging.info("Program terminated.")
