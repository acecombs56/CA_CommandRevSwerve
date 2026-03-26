#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from commands2 import PrintCommand, WaitCommand
from commands.launch import Launch
from commands.intake import Intake
from commands.launchstop import LaunchStop
from commands.trajectory import JerkyTrajectory
from subsystems.canfuelsubsystem import CANFuelSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from commands.reset_xy import ResetXY, ResetSwerveFront
from commands.gotopoint import GoToPoint
from commands.aimtodirection import AimToDirection
from commands.swervetopoint import SwerveToPoint

class CAFarLeftNeutral(commands2.SequentialCommandGroup):
    def __init__(self, fuelSubsystem: CANFuelSubsystem, drivetrain: DriveSubsystem) -> None:
        super().__init__()
        self.addCommands(
            PrintCommand("CA_Far Left Neutral started"),
            ResetXY(x=3.6, y=7.6, headingDegrees=0,drivetrain=drivetrain).withTimeout(.5),
            PrintCommand("Reset just ran"),

            SwerveToPoint(x=2.6 , y=7.6, headingDegrees=0, drivetrain=drivetrain, speed=1.0).withTimeout(2.0),
            AimToDirection(degrees=270, drivetrain=drivetrain, speed=1.0).withTimeout(1),
            GoToPoint(2.6, 4.0, drivetrain=drivetrain, speed=1.0),
            AimToDirection(degrees=0, drivetrain=drivetrain, speed=1.0).withTimeout(1),

            Launch(fuelSubsystem).withTimeout(3),
            WaitCommand(3.0).withTimeout(5.0),
            LaunchStop(fuelSubsystem).withTimeout(.5),
            PrintCommand("stop launch roller finished"),

            # return to left wall, go to neutral area, drive through fuel picking up

            AimToDirection(degrees=90, drivetrain=drivetrain, speed=1.0).withTimeout(1),
            GoToPoint(2.6, 7.4, drivetrain=drivetrain, speed=1.0),
            AimToDirection(degrees=0, drivetrain=drivetrain, speed=1.0).withTimeout(1),
            GoToPoint(7.6, 7.4, drivetrain=drivetrain, speed=1.0),
            AimToDirection(degrees=260, drivetrain=drivetrain, speed=1.0).withTimeout(1),

            # start intake  - need new commands to start intake motor , one to stop motor
            Intake(fuelSubsystem).withTimeout(1.0),
            # drive forward picking up fuel and hopefully pushing toward our side
            # GoToPoint(7.6, 4.2, drivetrain=drivetrain, speed=1.0),
            SwerveToPoint(x=7.6, y=3.9, headingDegrees=260, drivetrain=drivetrain, speed=1.0).withTimeout(5.0),

            # stop intake & motors at end

            PrintCommand("CA_Far Left Neutral finished")
        )