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
    def __init__(self, driveSubsystem: DriveSubsystem, fuelSubsystem: CANFuelSubsystem) -> None:
        super().__init__()
        self.addCommands(
            PrintCommand("CA_TestAuto started"),
            JerkyTrajectory(drivetrain=driveSubsystem, endpoint=(2.1, 7.4, 270)),
            Launch(fuelSubsystem).withTimeout(2),
            PrintCommand("CA_TestAuto finished")
        )