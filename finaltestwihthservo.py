
import random
import time
import copy
import signal
import sys
import numpy as np
# from neurapy.robot import Robot
# import quaternion

# Defining a signal handler for graceful interruption
def signal_handler(signum, frame):
    print("Signal handler called with signal", signum)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Defining a Class called Camera
class Camera:
    def __init__(self):
        self.item_detected = False
        self.item_position = None
        self.item_type = None

    def detect_item(self):
        self.item_detected = random.choice([True, False])
        if self.item_detected:
            self.item_position = (random.randint(0, 150), random.randint(0, 80))
            self.item_type = random.choice(["circle", "square", "rectangle", "oval", "star"])
        else:
            self.item_position = None
            self.item_type = None
        return self.item_detected, self.item_position, self.item_type

    def send_signal(self):
        if self.item_detected:
            print("Signal: Item detected, sending signal to Cobot1 to sort.")
            return True
        return False


# Defining a Class called Cobot  
class Cobot:
    def __init__(self, name):
        self.name = name
        self.is_busy = False

    def sort_item(self, position, item_type):
        if not self.is_busy:
            print(f"{self.name} is moving to sort the item at position {position}, identified as {item_type}.")
            self.is_busy = True
            time.sleep(4)
            print(f"{self.name} has sorted the item (type: {item_type}) and placed it in the conveyor belt for Cobot2.")
            self.is_busy = False
            return True
        else:
            print(f"{self.name} is currently busy sorting.")
            return False

    def pick_sorted_item(self, item_type, boxes):
        box_mapping = {
            "circle": "Box-a",
            "square": "Box-b",
            "rectangle": "Box-c",
            "oval": "Box-d",
            "star": "Box-e"
        }
        if not self.is_busy:
            print(f"{self.name} is moving to pick up the sorted item of type {item_type} from the sorted area.")
            self.is_busy = True
            time.sleep(4)
            print(f"{self.name} has picked the sorted item of type {item_type} from the sorted area.")

            box = box_mapping.get(item_type)
            if box:
                boxes[box].append(1)
                print(f"{self.name} has placed the item of type {item_type} in {box}.")
            else:
                print(f"Error: No box found for item type {item_type}.")
            self.is_busy = False


# Defining a Class called ConveyorTrackingSystem 
class ConveyorTrackingSystem:
    def __init__(self):
        self.camera = Camera()
        self.cobot1 = Cobot("Cobot 1 (Sorter)")
        self.cobot2 = Cobot("Cobot 2 (Picker)")
        self.sorted_items = []
        self.boxes = {
            "Box-a": [],
            "Box-b": [],
            "Box-c": [],
            "Box-d": [],
            "Box-e": []
        }
        # self.robot = Robot()  # Uncomment if Robot is available

    def run(self):
        iteration = 0
        # self.robot.activate_servo_interface('position')  # Uncomment if Robot is available

        target_1 = [0.3, 0.25, 0.1]
        target_2 = [0.25, 0.3, 0.2]

        while iteration < 5:
            print("Camera scanning for items on the conveyor belt")
            item_detected, position, item_type = self.camera.detect_item()
            print("##############################################################")
            if item_detected:
                print(f"Item detected at position {position}, identified as {item_type}.")

                if self.camera.send_signal():
                    if not self.cobot1.is_busy:
                        sorting_done = self.cobot1.sort_item(position, item_type)
                        if sorting_done:
                            self.sorted_items.append(item_type)
                            if not self.cobot2.is_busy and self.sorted_items:
                                self.cobot2.pick_sorted_item(self.sorted_items.pop(0), self.boxes)

                                for target in [target_1, target_2]:
                                    print("Moving robot to target:", target)
                                    # Uncomment the following lines if Robot is available
                                    # current_pose = copy.deepcopy(self.robot.get_current_cartesian_pose())
                                    # for i in range(3):
                                    #     current_pose[i] += target[i]
                                    # velocity = [0.15] * 6
                                    # acceleration = [2.0] * 6
                                    # error_code = self.robot.movelinear_online(current_pose, velocity, acceleration)
                                    time.sleep(2)  # Simulate the movement time
                                    target = target[:]  # Copy target to avoid in-place modification issues

            else:
                print("No item detected.")

            time.sleep(4)
            iteration += 1

        # Uncomment if Robot is available
        # self.robot.stop_movelinear_online()
        # print("Robot stopped")
        # time.sleep(2)
        # self.robot.deactivate_servo_interface()
        # self.robot.stop()

        print("\nFinal Box Contents:")
        for item_type, box in self.boxes.items():
            print(f"{item_type.capitalize()} box contains: {len(box)} items.")


# Running the conveyor tracking system
if __name__ == "__main__":
    tracking_system = ConveyorTrackingSystem()
    tracking_system.run()