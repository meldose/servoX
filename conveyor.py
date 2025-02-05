
class TargetPositionCalculator:
    def __init__(self, RX, RY, RZ, AMSD, delta_TSC, delta_CTO):
        self.RX = RX  # Initial target X position (detected by scanner)
        self.RY = RY  # Initial target Y position
        self.RZ = RZ  # Initial target Rotation (Z axis)
        self.AMSD = AMSD  # Marker space displacement
        self.delta_TSC = delta_TSC  # Time-based displacement due to scanning delay
        self.delta_CTO = delta_CTO  # Conveyor tracking correction
    
    def calculate_target_position(self):
        TD = self.delta_TSC + self.delta_CTO  # Total displacement correction
        TX = self.RX + self.AMSD + TD  # Adjusted X position
        TY = self.RY  # Y position remains unchanged
        TRZ = self.RZ  # Rotation remains unchanged
        return TX, TY, TRZ

# Example usage with sample values
RX = 150   # Scanner detected X position
RY = 100   # Scanner detected Y position
RZ = 30    # Scanner detected Rotation (degrees)
AMSD = 10  # Marker space displacement
delta_TSC = 5   # Displacement due to scanning delay
delta_CTO = 2   # Conveyor tracking correction

calculator = TargetPositionCalculator(RX, RY, RZ, AMSD, delta_TSC, delta_CTO)
TX, TY, TRZ = calculator.calculate_target_position()

print(f"Final Target Position: X = {TX}, Y = {TY}, RZ = {TRZ}")
