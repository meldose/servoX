import time # import time module
import random # importing random module

# Defining a class called ServoX
class ServoX:
    def __init__(self): # initialize the class
        self.angle = 0  # initial servo angle
    
    
    # Defining a function called move_to_angle
    def move_to_angle(self, angle): # defining function to move to angle
        print(f"Moving servo to {angle} degrees.")
        self.angle = angle # set angle
        # Code to move the servo motor to the specified angle
        time.sleep(1)  # Simulating the time it takes for the servo to move

# create an class for conveyor belt
class ConveyorBelt: # define a class for conveyor belt
    def __init__(self): # initialize the class 
        self.speed = 1  # Conveyor speed

    def start(self): # defining the function to start the conveyor belt
        print("Conveyor belt started.")
        # Code to start conveyor belt

    def stop(self):
        print("Conveyor belt stopped.")
        # Code to stop conveyor belt

    def get_object_position(self):
        # Simulating the detection of objects at random intervals on the conveyor
        return random.randint(0, 180)

# Defining an function for conveyor tracking
def conveyor_tracking(servo, conveyor): # define a function for conveyor tracking
    conveyor.start() # conveyor start
    
    while True: # set inifinite loop using while
        # Simulate detecting object on the conveyor
        object_position = conveyor.get_object_position() # set object position
        print(f"Detected object at position {object_position} on the conveyor.") # print message
        
        # Move the servo to the detected object position
        servo.move_to_angle(object_position)
        
        # Add condition to stop after a few iterations for simulation
        time.sleep(2) # set time to 2
        conveyor.stop() # stop conveyor
        break

# Instantiate objects
servo = ServoX() # set servo variable to ServoX class
conveyor = ConveyorBelt() # set conveyor variable to ConveyorBelt class

# Start tracking objects on the conveyor
conveyor_tracking(servo, conveyor) # calling the function with servo and conveyor
