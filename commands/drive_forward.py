#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations
import commands2

from subsystems.drivesubsystem import DriveSubsystem


class DriveForward(commands2.Command):
    """
    A simple command that drives the robot forward a specified distance using odometry.
    The robot drives forward (robot-relative) at the given speed and stops after traveling
    the desired distance.
    """

    def __init__(self, distanceMeters: float, speed: float, drivetrain: DriveSubsystem) -> None:
        """
        :param distanceMeters: How far to drive forward in meters (e.g. 1.0 for one meter).
        :param speed: Normalized forward speed (0.0 to 1.0), fraction of max speed.
        :param drivetrain: The drive subsystem used by this command.
        """
        super().__init__()
        self.distanceMeters = distanceMeters
        self.speed = speed
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self.startPosition = None

    def initialize(self):
        self.startPosition = self.drivetrain.getPose().translation()

    def execute(self):
        # Drive forward robot-relative, no strafing, no rotation
        self.drivetrain.drive(self.speed, 0, 0, fieldRelative=False, rateLimit=True, square=False)

    def isFinished(self) -> bool:
        currentPosition = self.drivetrain.getPose().translation()
        distanceTraveled = currentPosition.distance(self.startPosition)
        return distanceTraveled >= self.distanceMeters

    def end(self, interrupted: bool):
        self.drivetrain.drive(0, 0, 0, fieldRelative=False, rateLimit=False)
