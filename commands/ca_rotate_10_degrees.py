from __future__ import annotations
import commands2
from commands.aimtodirection import AimToDirection
from subsystems.drivesubsystem import DriveSubsystem


class CARotate10Degrees(commands2.Command):
    """
    Rotates the robot 10 degrees clockwise (positive) from its current heading.
    Can be modified to rotate counter-clockwise by changing the sign of the angle.
    """
    
    def __init__(self, drivetrain: DriveSubsystem, clockwise: bool = True):
        """
        :param drivetrain: The swerve drive subsystem
        :param clockwise: True for clockwise rotation (positive degrees), False for counter-clockwise
        """
        super().__init__()
        self.drivetrain = drivetrain
        self.rotationAmount = 15.0 if clockwise else -15.0
        self.targetHeading = None
        self.subcommand = None
        self.addRequirements(drivetrain)
    
    def initialize(self):
        # Get current heading and add 10 degrees to it
        currentHeading = self.drivetrain.getHeading().degrees()
        self.targetHeading = currentHeading + self.rotationAmount
        
        # Normalize the heading to be between -180 and 180 degrees
        while self.targetHeading > 180:
            self.targetHeading -= 360
        while self.targetHeading < -180:
            self.targetHeading += 360
        
        # Use the existing AimToDirection command to rotate to the target heading
        self.subcommand = AimToDirection(
            degrees=self.targetHeading,
            drivetrain=self.drivetrain,
            speed=0.5  # Adjust speed as needed (0.0 to 1.0)
        )
        self.subcommand.initialize()
    
    def execute(self):
        if self.subcommand:
            self.subcommand.execute()
    
    def isFinished(self) -> bool:
        if self.subcommand:
            return self.subcommand.isFinished()
        return False
    
    def end(self, interrupted: bool):
        if self.subcommand:
            self.subcommand.end(interrupted)