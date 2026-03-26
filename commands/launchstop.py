#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import wpilib

from constants import FuelConstants
from subsystems.canfuelsubsystem import CANFuelSubsystem


class LaunchStop(commands2.Command):
    def __init__(self, fuelSubsystem: CANFuelSubsystem) -> None:
        super().__init__()
        self.fuelSubsystem = fuelSubsystem
        self.addRequirements(self.fuelSubsystem)

    # def initialize(self) -> None:
        # DC CA do nothing except stop launcher motors

    def end(self, interrupted: bool) -> None:
        self.fuelSubsystem.setIntakeLauncherRoller(0)
        self.fuelSubsystem.setFeederRoller(0)

    def isFinished(self) -> bool:
        return False
