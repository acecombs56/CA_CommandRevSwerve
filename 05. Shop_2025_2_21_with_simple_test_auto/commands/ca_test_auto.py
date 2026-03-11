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


class CATestAuto(commands2.SequentialCommandGroup):
    def __init__(self, fuelSubsystem: CANFuelSubsystem, drivetrain: DriveSubsystem) -> None:
        super().__init__()
        self.addCommands(
            PrintCommand("CA_TestAuto started"),
            Launch(fuelSubsystem).withTimeout(2),
            JerkyTrajectory(
                drivetrain=drivetrain,
                speed=+1.0,
                waypoints=[],
                endpoint=(2.0, 7.4, 270),
            ),
            PrintCommand("CA_TestAuto finished")
        )