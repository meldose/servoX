import time # import time module
import random # importing random module

# Defining a class called ServoX
class ServoX:
    def __init__(self):
        self.angle = 0  # initial servo angle
    
    
    # Defining a function called move_to_angle
    def move_to_angle(self, angle):
        print(f"Moving servo to {angle} degrees.")
        self.angle = angle
        # Code to move the servo motor to the specified angle
        time.sleep(1)  # Simulating the time it takes for the servo to move

# create an class for conveyor belt
class ConveyorBelt: 
    def __init__(self):
        self.speed = 1  # Conveyor speed

    def start(self):
        print("Conveyor belt started.")
        # Code to start conveyor belt

    def stop(self):
        print("Conveyor belt stopped.")
        # Code to stop conveyor belt

    def get_object_position(self):
        # Simulating the detection of objects at random intervals on the conveyor
        return random.randint(0, 180)

# Defining an function for conveyor tracking
def conveyor_tracking(servo, conveyor):
    conveyor.start()
    
    while True:
        # Simulate detecting object on the conveyor
        object_position = conveyor.get_object_position()
        print(f"Detected object at position {object_position} on the conveyor.")
        
        # Move the servo to the detected object position
        servo.move_to_angle(object_position)
        
        # Add condition to stop after a few iterations for simulation
        time.sleep(2)
        conveyor.stop()
        break

# Instantiate objects
servo = ServoX()
conveyor = ConveyorBelt()

# Start tracking objects on the conveyor
conveyor_tracking(servo, conveyor)
