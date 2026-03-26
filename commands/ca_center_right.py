#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from commands2 import PrintCommand, WaitCommand
from commands.launch import Launch
from commands.launchstop import LaunchStop
from commands.trajectory import JerkyTrajectory
from subsystems.canfuelsubsystem import CANFuelSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from commands.reset_xy import ResetXY, ResetSwerveFront
from commands.gotopoint import GoToPoint
from commands.aimtodirection import AimToDirection
from commands.swervetopoint import SwerveToPoint
from commands.drive_forward import DriveForward
from commands.ca_rotate_30_degrees import CARotate30Degrees


class CACenterRight(commands2.SequentialCommandGroup):
    def __init__(self, fuelSubsystem: CANFuelSubsystem, drivetrain: DriveSubsystem) -> None:
        super().__init__()
        self.addCommands(
            PrintCommand("CA_Center Left started"),
            ResetXY(x=3.56, y=4.9, headingDegrees=0,drivetrain=drivetrain).withTimeout(.5),
            PrintCommand("Reset just ran"),
            DriveForward(distanceMeters=.65, speed=0.5, drivetrain=drivetrain).withTimeout(3),
            # SwerveToPoint(x=2.5 , y=4.9, headingDegrees=330, drivetrain=drivetrain, speed=1.0).withTimeout(3.0),
            # AimToDirection(degrees=150, drivetrain=drivetrain, speed=.10).withTimeout(5),
            CARotate30Degrees(drivetrain=drivetrain,clockwise=False).withTimeout(.5),
            Launch(fuelSubsystem).withTimeout(6),
            WaitCommand(8.0).withTimeout(5.0),
            LaunchStop(fuelSubsystem).withTimeout(.5),
            PrintCommand("stop launch roller finished"),

            # WaitCommand(3.0).withTimeout(3.0),

            # AimToDirection(degrees=90, drivetrain=drivetrain, speed=1.0).withTimeout(1),
            # GoToPoint(3.0, 7.4, drivetrain=drivetrain, speed=1.0),
            # AimToDirection(degrees=0, drivetrain=drivetrain, speed=1.0).withTimeout(1),

            PrintCommand("CA_Center Left finished")
        )