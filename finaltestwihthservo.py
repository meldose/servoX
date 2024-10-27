# from neurapy.robot import robot
import random # import libraries to select random items
import time # imported time libraries
import copy # import copy module
import signal # import signal module
import sys # import sys module
import numpy as np
# import quaternion

# Defining a signal handler for graceful interruption
def signal_handler(signum, frame):
    print("Signal handler called with signal", signum)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Importing the Robot class
from neurapy.robot import Robot

# Defining a Class called Camera
class Camera:
    def __init__(self):
        self.item_detected = False  # No item detected initially
        self.item_position = None
        self.item_type = None  # This could represent size, color, or other criteria

    # Defining a function called detect item
    def detect_item(self):
        # Randomly simulate item detection
        self.item_detected = random.choice([True])
        if self.item_detected:  # if item is detected then trigger a signal
            self.item_position = (random.randint(0, 150), random.randint(0, 80))  # Random position on conveyor
            self.item_type = random.choice(["circle", "square", "rectangle", "oval", "star"])  # Simulate item types
        else:
            self.item_position = None
            self.item_type = None
        return self.item_detected, self.item_position, self.item_type

    # Signal method to notify Cobot1
    def send_signal(self):
        if self.item_detected:
            print("Signal: Item detected, sending signal to Cobot1 to sort.")
            return True  # Signal to Cobot1
        return False


# Defining a Class called Cobot  
class Cobot:
    def __init__(self, name):
        self.name = name
        self.is_busy = False  # Set cobot to not busy initially

    def sort_item(self, position, item_type): # defining sort_item method function
        if not self.is_busy: #check if the cobot is not busy
            print(f"{self.name} is moving to sort the item at position {position}, identified as {item_type}.") # printing message
            self.is_busy = True # set cobot to busy
            time.sleep(4)  # Simulate sorting time
            print(f"{self.name} has sorted the item (type: {item_type}) and placed it in the conveyor belt for Cobot2.") # print message
            self.is_busy = False  # set cobot to not busy
            return True  # Sorting done
        else:
            print(f"{self.name} is currently busy sorting.") #show message
            return False # sorting not done

    def pick_sorted_item(self, item_type, boxes): # defining pick_sorted_item method function
        box_mapping = {  # Mapping between item types and box names
            "circle": "Box-a",
            "square": "Box-b",
            "rectangle": "Box-c",
            "oval": "Box-d",
            "star": "Box-e"
        }
        if not self.is_busy:  # if Cobot2 is not busy then pick item
            print(f"{self.name} is moving to pick up the sorted item of type {item_type} from the sorted area.") #print message
        self.is_busy = True # set cobot to busy
        time.sleep(4)  # Simulate picking up sorted item
        print(f"{self.name} has picked the sorted item of type {item_type} from the sorted area.") # print message

        # Place the picked item in the correct box
        box = box_mapping.get(item_type) # serch for box name based on item type
        if box: # if box is found
            boxes[box].append(1)  # Add the item to the corresponding box
            print(f"{self.name} has placed the item of type {item_type} in {box}.") # print message
        else:
            print(f"Error: No box found for item type {item_type}.") # print error message
        self.is_busy = False # set cobot to not busy


# Defining a Class called ConveyorTrackingSystem 
class ConveyorTrackingSystem: # defining class called ConveyorTrackingSystem
    def __init__(self): # initialize the class
        self.camera = Camera() #setup camera
        self.cobot1 = Cobot("Cobot 1 (Sorter)") # setup cobots
        self.cobot2 = Cobot("Cobot 2 (Picker)") # setup cobots
        self.sorted_items = []  # List to track sorted items ready for pickup
        self.boxes = {
            "Box-a": [],
            "Box-b": [],
            "Box-c": [],
            "Box-d": [],
            "Box-e": []
        }  # Dictionary to hold boxes for different item types
        self.robot = Robot()  # Initialize the neurapy robot

    def run(self): # defining run function
        iteration = 0 # set iteration to 0
        self.robot.activate_servo_interface('position')  # Activate servo interface 

        # Define targets for testing robot movement
        target_1 = [0.3, 0.25, 0.1]  # Target position 1 #set target position1
        target_2 = [0.25, 0.3, 0.2]  # Target position 2 # set target position2

        while iteration < 5:  # Add a condition to stop after 5 iterations
            print("Camera scanning for items on the conveyor belt") # print message
            item_detected, position, item_type = self.camera.detect_item() #se detect item from camera
            print("##############################################################") 
            if item_detected: # if item is detected
                print(f"Item detected at position {position}, identified as {item_type}.") #show message

                # Send signal from Camera to Cobot1
                if self.camera.send_signal(): # check if signal is sent
                    # Assign Cobot1 to sort the item based on the signal
                    if not self.cobot1.is_busy: # check if Cobot1 is not busy
                        sorting_done = self.cobot1.sort_item(position, item_type) # call sort_item method
                        if sorting_done: # if sorting is done
                            # Add the sorted item to the list of items to be picked
                            self.sorted_items.append(item_type) # set item to be picked
                            # Cobot2 picks the item immediately after it's sorted
                            if not self.cobot2.is_busy and self.sorted_items: # check if Cobot2 is not busy
                                # Cobot2 only picks after Cobot1 has sorted
                                self.cobot2.pick_sorted_item(self.sorted_items.pop(0), self.boxes)

                                # Move the robot between target positions
                                for target in [target_1, target_2]: # for loop to move the robot between target positions
                                    print("Moving robot to target:", target) # print message
                                    current_pose = copy.deepcopy(self.robot.get_current_cartesian_pose()) # set current pose as variable and copy the robot pose
                                    current_pose[:3] += target # increase the robot position
                                    velocity = [0.15] * 6 # set velocity to 0.15
                                    acceleration = [2.0] * 6 #set acceleration to 2
                                    error_code = self.robot.movelinear_online(current_pose, velocity, acceleration) # set error code
                                    time.sleep(10) # put the robot to sleep for 10 seconds
                                    target[0] -= target_1[0]  # Adjust target as part of simulation

            else:
                print("No item detected.") # print message

            # Wait before the next camera scan
            time.sleep(4) # put the robot to sleep for 4 seconds
            iteration += 1  # Increment the iteration counter

        self.robot.stop_movelinear_online()  # Stop the robot movement
        print("Robot stopped") # print message
        time.sleep(2) #set time to sleep for 2 seconds
        self.robot.deactivate_servo_interface()  # Deactivate the servo interface
        self.robot.stop()  # Stop the robot

        # Print the contents of the boxes at the end
        print("\nFinal Box Contents:") # print message
        for item_type, box in self.boxes.items(): #check for item type and box
            print(f"{item_type.capitalize()} box contains: {len(box)} items.")


# Running the conveyor tracking system
if __name__ == "__main__": # check if the code is run as the main program
    tracking_system = ConveyorTrackingSystem() #setup tracking system
    tracking_system.run() # call run function and run the system

