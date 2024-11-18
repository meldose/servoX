
import time
import logging
import signal
import sys
import argparse
from neurapy.robot import Robot

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

# Global flag for graceful shutdown
running = True

def signal_handler(sig, frame):
    global running
    logging.info('Interrupt received, stopping servo tracking...')
    running = False

# Register the signal handler for graceful shutdown
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# ============================
# Argument Parsing
# ============================

def parse_arguments():
    parser = argparse.ArgumentParser(description='Servo Tracking with Conveyor')
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
# Robot Initialization
# ============================

def initialize_robot(config_path):
    try:
        robot = Robot(config_path=config_path)
        logging.info(f"Robot initialized with config: {config_path}")
        # Optionally, verify connection here if Robot class supports it
        if hasattr(robot, 'is_connected'):
            if not robot.is_connected():
                logging.error("Robot is not connected. Please check the connection.")
                sys.exit(1)
            else:
                logging.info("Robot connection verified.")
        return robot
    except FileNotFoundError:
        logging.error(f"Configuration file not found at: {config_path}")
        sys.exit(1)
    except Exception as e:
        logging.error(f"Failed to initialize Robot: {e}")
        sys.exit(1)

# ============================
# Servo Tracking Function
# ============================

def servo_tracking_with_conveyor(robot, conveyor_speed, time_interval, tracking_duration, acceleration, velocity):
    start_time = time.time()
    while running and (time.time() - start_time) < tracking_duration:
        try:
            current_position = robot.get_tcp_position()
            if current_position is None:
                logging.warning("Current position is None. Check robot status.")
                break

            # Ensure the position is a mutable list
            new_position = list(current_position)
            new_position[0] += conveyor_speed * time_interval  # Update X position

            # Optional: Validate new_position within robot's workspace
            if hasattr(robot, 'is_position_within_workspace'):
                if not robot.is_position_within_workspace(new_position):
                    logging.warning(f"New position {new_position} is out of workspace bounds.")
                    break

            # Execute servo movement with acceleration and velocity
            robot.servox(new_position, a=acceleration, v=velocity)
            logging.debug(f"Moved to position: {new_position}")

            time.sleep(time_interval)
        except AttributeError as ae:
            logging.error(f"Attribute Error: {ae}")
            break
        except Exception as e:
            logging.exception("Error during servo tracking")
            break

    logging.info("Servo tracking completed.")

# ============================
# Main Function
# ============================

def main():
    args = parse_arguments()

    logging.info("Starting Servo Tracking with Conveyor")
    logging.info(f"Parameters: Conveyor Speed={args.conveyor_speed} m/s, "
                 f"Time Interval={args.time_interval} s, "
                 f"Tracking Duration={args.tracking_duration} s, "
                 f"Acceleration={args.acceleration}, "
                 f"Velocity={args.velocity}")

    robot = initialize_robot(args.config_path)
    servo_tracking_with_conveyor(
        robot=robot,
        conveyor_speed=args.conveyor_speed,
        time_interval=args.time_interval,
        tracking_duration=args.tracking_duration,
        acceleration=args.acceleration,
        velocity=args.velocity
    )

# ============================
# Entry Point
# ============================

if __name__ == "__main__":
    main()

