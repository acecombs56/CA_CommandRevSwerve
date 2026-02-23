#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import wpilib

from constants import FuelConstants
from subsystems.canfuelsubsystem import CANFuelSubsystem


class Launch(commands2.Command):
    def __init__(self, fuelSubsystem: CANFuelSubsystem) -> None:
        super().__init__()
        self.fuelSubsystem = fuelSubsystem
        self.addRequirements(self.fuelSubsystem)
        print(f"_Init_ of launch.py")

    def initialize(self) -> None:
        print(f"Init of launch.py")
        self.fuelSubsystem.setIntakeLauncherRoller(
            wpilib.SmartDashboard.getNumber(
                "Launching launcher roller value",
                FuelConstants.LAUNCHING_LAUNCHER_VOLTAGE,
            )
        )
        self.fuelSubsystem.setFeederRoller(
            wpilib.SmartDashboard.getNumber(
                "Launching feeder roller value", FuelConstants.LAUNCHING_FEEDER_VOLTAGE
            )
        )

    def isFinished(self) -> bool:
        print(f"End of launch.py")
        return False
