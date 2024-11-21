module ObjectMessages:

    class NewBoxObjectMsg:
        def __init__(self):
            self.position = [0.0, 0.0, 0.0]  # sequence<double, 3>
            self.quaternion = [0.0, 0.0, 0.0, 0.0]  # sequence<double, 4>
            self.uuid = [0] * 16  # sequence<octet, 16>
            self.dimensions = [0.0, 0.0, 0.0]  # sequence<double, 3>

    class KnownObjectPoseMsg:
        def __init__(self):
            self.uuid = [0] * 16  # sequence<octet, 16>
            self.position = [0.0, 0.0, 0.0]  # sequence<double, 3>
            self.quaternion = [0.0, 0.0, 0.0, 0.0]  # sequence<double, 4>

    class PickingState:
        UNKNOWN = 0
        UNAVAILABLE = 1
        MOVING = 10
        REACHED = 20
        PICKED = 30
        PICK_FAILED = 40

    class PlacingState:
        UNKNOWN = 0
        UNAVAILABLE = 1
        OBJECT_ID_UNKNOWN = 2
        MOVING = 10
        REACHED = 20
        RELEASED = 30
        RELEASE_FAILED = 40
