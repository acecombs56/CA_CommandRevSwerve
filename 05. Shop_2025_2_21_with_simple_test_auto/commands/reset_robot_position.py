from __future__ import annotations
import commands2

from wpimath.geometry import Rotation2d, Pose2d, Translation2d


class ResetRobotPosition(commands2.Command):
    def __init__(self, drivetrain):
        """
        Reset the robot's position and rotation to a known state: x=3, y=3, rotation=0 degrees.
        :param drivetrain: DriveSubsystem on which the odometry should be reset
        """
        super().__init__()
        self.drivetrain = drivetrain
        self.position = Pose2d(Translation2d(3, 3), Rotation2d.fromDegrees(0))
        self.addRequirements(drivetrain)

    def initialize(self):
        self.drivetrain.resetOdometry(self.position)

    def isFinished(self) -> bool:
        return True  # this is an instant command, it finishes right after it initialized

    def execute(self):
        """
        nothing to do here, this is an instant command
        """

    def end(self, interrupted: bool):
        """
        nothing to do here, this is an instant command
        """
