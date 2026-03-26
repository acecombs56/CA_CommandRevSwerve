#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from commands2 import PrintCommand
from commands.launch import Launch
from commands.trajectory import JerkyTrajectory
from subsystems.canfuelsubsystem import CANFuelSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from commands.reset_xy import ResetXY, ResetSwerveFront
from commands.gotopoint import GoToPoint
from commands.aimtodirection import AimToDirection
from commands.swervetopoint import SwerveToPoint
from commands.drive_forward import DriveForward

class CATestAuto(commands2.SequentialCommandGroup):
    def __init__(self, fuelSubsystem: CANFuelSubsystem, drivetrain: DriveSubsystem) -> None:
        super().__init__()
        self.addCommands(
            ResetXY(x=3.6, y=7.6, headingDegrees=180, drivetrain=drivetrain).withTimeout(.5),
            PrintCommand("CA_TestAuto started"),
            DriveForward(distanceMeters=1.0, speed=1.0, drivetrain=drivetrain).withTimeout(2),

            # SwerveToPoint(2.1, 7.6, headingDegrees=270, drivetrain=drivetrain, speed=1.0),
            AimToDirection(degrees=270, drivetrain=drivetrain, speed=0.5).withTimeout(3),
            DriveForward(distanceMeters=2.0, speed=0.5, drivetrain=drivetrain).withTimeout(4),
            GoToPoint(2.1, 3.6, drivetrain=drivetrain, speed=1.0),
            AimToDirection(degrees=0, drivetrain=drivetrain, speed=0.5).withTimeout(3),

           # JerkyTrajectory( drivetrain=drivetrain, speed=+1.0,  waypoints=[],
           #     endpoint=(2.0, 7.4, 270), ),
           # JerkyTrajectory(drivetrain=drivetrain, speed=+1.0, waypoints=[],
           #                 endpoint=(3.0, 6.4, 0), ),
           # Launch(fuelSubsystem).withTimeout(2),
            PrintCommand("CA_TestAuto finished")
        )