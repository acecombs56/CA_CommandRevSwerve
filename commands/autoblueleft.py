#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from commands2 import PrintCommand
from commands.launch import Launch
from subsystems.canfuelsubsystem import CANFuelSubsystem


#  from subsystems.candrivesubsystem import CANDriveSubsystem
# from subsystems.canfuelsubsystem import CANFuelSubsystem

#  fuelSubsystem = CANFuelSubsystem()
# DC CA removed parm1  fuelSubsystem : CANFuelSubsystem

class AutoBlueLeft(commands2.SequentialCommandGroup):
    def __init__(
        self ,
    ) -> None:
        super().__init__()
        self.addCommands(

            PrintCommand(f"AutoBlueLeft was selected"),
            Launch(CANFuelSubsystem).withTimeout(10),
        )


