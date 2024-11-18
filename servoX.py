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
    # --------------------------  CREATING Robot & Program & RobotStatus OBJECTS  -----------------------
    program_handler = Program(robot_handler)
    robot_status = RobotStatus(robot_handler)
    iterator = loopCount()
    tool_objects = {}

    sio_object = register_sio_callbacks(program_handler)

    id_manager = CmdIDManager(3)

    # Initializing local tools for Program 'Program_001'
    tool_objects['NoTool'] = robot_handler.gripper(gripper_name='STANDARD_GRIPPER', tool_name='NoTool')
    # Setting tool 'NoTool' for Program 'Program_001'
    try:
        current_tool = 'NoTool'
        current_tool_params = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        robot_handler.set_tool(tool_name=current_tool, tool_params=current_tool_params)
    except Exception as e:
        program_handler._Program__PS.socket_object.send_gui_message(f'Error {current_tool} tool not set properly: {str(e)}', 'Error')
        raise e

    # -------------------------------  INITIAL JOINT STATE & CARTESIAN POSE  ----------------------------
    previous_joint_angles = robot_status.getRobotStatus('jointAngles')

    # --------------------------------------  PLANNING COMMANDS  ----------------------------------------
    # Planning motion for ServoX and ServoJ
    motion_data = {
        "speed": 50.0,
        "acceleration": 50.0,
        "target_joint": [
            previous_joint_angles,
            [
                1.5708,  # Angle for ServoX
                0.0,     # Angle for ServoJ
                -2.49582083,
                0,
                0,
                0
            ]
        ]
    }
    # Setting command for joint movement
    program_handler.set_command(cmd.Joint, **motion_data, cmd_id=2, current_joint_angles=previous_joint_angles, reusable_id=1)

    # --------------------------------------  EXECUTING COMMANDS  ---------------------------------------
    block_until_next_step(robot_handler)
    program_handler.execute([2])
    sio_object.send_gui_message({"state": "FINISHED"}, socket_name="StepByStep")

def register_sio_callbacks(program_handler: Program):
    sio_handler = get_sio_client_singleton_instance()
    sio_register = sio_handler.get_sio_client_register_obj()
    sio_register.on("StepByStep", handle_sio_step_by_step)
    sio_obj_client = sio_handler.get_sio_client_obj()
    return sio_obj_client

def handle_sio_step_by_step(data: dict):
    global cur_step
    global is_new_step
    print(f"handling socket message on channel 'StepByStep' with data {data}")
    old_step = cur_step
    cur_step = int(data.get('id', -1))
    if(old_step != cur_step):
        is_new_step = True

def read_cur_step_from_gui() -> int:
    global cur_step
    return cur_step

def block_until_next_step(robot: Robot):
    global cur_step
    global is_new_step
    old_step = cur_step
    while(robot.is_robot_in_semi_automatic_mode() and cur_step <= old_step and not is_new_step):
        cur_step = read_cur_step_from_gui()
        sleep(STEP_SLEEP_INTERVAL)
    is_new_step = False

def signal_handler(signum, frame):
    hr.callService('ResetPythonProgramStatus',[])
    rts.callService('StopPythonScript', [])
    sys.exit()

if __name__ == "__main__":
    robot_handler = Robot()
    program_handler = Program(robot_handler)
    rts = Component(robot_handler, "RTS")
    hr = Component(robot_handler, "HR")
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    try:
        main(robot_handler)
    except Exception as e:
        print('Exception:', str(e.msg if hasattr(e, 'msg') else e))
        hr.callService('ResetPythonProgramStatus', [])
        program_handler._Program__PS.socket_object.send_gui_message('Error ' + str(e.msg if hasattr(e, 'msg') else e))
    finally:
        robot_handler.set_tool(tool_name='NoTool', tool_params=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

