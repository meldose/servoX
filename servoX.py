# # Servo control to track a part on the conveyor in the X-axis.
# from neurapy.robot import Robot
# conveyor_speed = 0.2  # Conveyor speed in meters per second.
# time_interval = 0.1   # Time interval in seconds for each update.
# r=Robot()
# # Function to track the object moving along X with the conveyor
# def servo_tracking_with_conveyor():
#     while True:
#         current_position = r.get_current_position()  # Get current position of TCP
#         new_position = current_position
#         new_position[0] += conveyor_speed * time_interval  # Update X position with conveyor speed
#         servox(new_position, a=1.2, v=0.25)  # Execute servo movement with given acceleration and velocity
#         wait(time_interval)  # Wait for the next update step

# servo_tracking_with_conveyor() # calling the function to track the conveyor


from neurapy.robot import Robot
import time

conveyor_speed = 0.2  # Conveyor speed in meters per second
time_interval = 0.1   # Time interval in seconds for each update
tracking_duration = 10  # Total time to track the object in seconds

# Initialize the Robot with the correct configuration path if necessary
try:
    r = Robot()  # Add config_path='/path/to/config.param' if needed
except Exception as e:
    print(f"Failed to initialize Robot: {e}")
    exit(1)

# Function to track the object moving along X with the conveyor
def servo_tracking_with_conveyor():
    start_time = time.time()
    while time.time() - start_time < tracking_duration:
        try:
            # Use the correct method to get the current TCP position
            current_position = r.get_tcp_position()  # Updated method name
            if current_position is None:
                print("Current position is None. Check robot status.")
                break

            new_position = current_position.copy()  # Ensure a copy is made
            new_position[0] += conveyor_speed * time_interval  # Update X position

            # Execute servo movement with acceleration and velocity
            r.servox(new_position, a=1.2, v=0.25)  # Assuming servox is a method of Robot
            time.sleep(time_interval)  # Wait for the next update step
        except AttributeError as ae:
            print(f"Attribute Error: {ae}")
            break  # Exit the loop on attribute error
        except Exception as e:
            print(f"Error during servo tracking: {e}")
            break  # Exit the loop on other errors

    print("Servo tracking completed.")

# Start the tracking
servo_tracking_with_conveyor()
