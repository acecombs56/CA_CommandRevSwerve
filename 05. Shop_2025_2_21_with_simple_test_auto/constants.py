# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""


import math

from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians

import rev
from rev import SparkBase, SparkBaseConfig, ClosedLoopConfig, FeedbackSensor


class NeoMotorConstants:
    kFreeSpeedRpm = 5676


class DriveConstants:
    # Driving Parameters - Note that these are not the maximum capable speeds of
    # the robot, rather the allowed maximum speeds
    kMaxSpeedMetersPerSecond = 3.0     # DC CA was 4.8
    kMaxAngularSpeed = 2 * math.tau  # radians per second

    kMagnitudeSlewRate = 9.8  # m/s per second (1g acceleration allowed along X and along Y axis)
    kRotationalSlewRate = 12.0  # rad/s per second

    # Chassis configuration
    kTrackWidth = units.inchesToMeters(25.5)   # DC CA was 26.5
    # Distance between centers of right and left wheels on robot
    kWheelBase = units.inchesToMeters(25.5)

    # Distance between front and back wheels on robot
    kModulePositions = [
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    ]
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    # set it to True if you were using a ruler for zeroing and want to ignore the offsets below
    kAssumeZeroOffsets = False   # DC was true True
 # DC CA trying to get wheels adjusted by 90 degrees only when rotating
    # set the above to == False, if you are using Rev zeroing tool (and you have to tinker with offsets below)
    kFrontLeftChassisAngularOffset = -math.pi / 2
    kFrontRightChassisAngularOffset = math.pi / 2
    kBackLeftChassisAngularOffset = math.pi / 2
    kBackRightChassisAngularOffset = math.pi / 2

    # SPARK MAX CAN IDs
    # DC CA updated can ids for CA 2025 robot
    kFrontLeftDrivingCanId = 11
    kRearLeftDrivingCanId = 13
    kFrontRightDrivingCanId = 12
    kRearRightDrivingCanId = 14

    kFrontLeftTurningCanId = 21
    kRearLeftTurningCanId = 23
    kFrontRightTurningCanId = 22
    kRearRightTurningCanId = 24

    kGyroReversed = -1  # can be +1 if not flipped (affects field-relative driving)


def getSwerveDrivingMotorConfig() -> SparkBaseConfig:
    drivingConfig = SparkBaseConfig()
    drivingConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    drivingConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
    drivingConfig.encoder.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
    drivingConfig.encoder.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor)
    drivingConfig.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
    drivingConfig.closedLoop.pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
    drivingConfig.closedLoop.velocityFF(ModuleConstants.kDrivingFF)
    drivingConfig.closedLoop.outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput)
    return drivingConfig


def getSwerveTurningMotorConfig(turnMotorInverted: bool) -> SparkBaseConfig:
    turningConfig = SparkBaseConfig()
    turningConfig.inverted(turnMotorInverted)
    turningConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    turningConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)
    turningConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
    turningConfig.absoluteEncoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)
    turningConfig.absoluteEncoder.inverted(ModuleConstants.kTurningEncoderInverted)
    turningConfig.closedLoop.setFeedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    turningConfig.closedLoop.pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
    turningConfig.closedLoop.velocityFF(ModuleConstants.kTurningFF)
    turningConfig.closedLoop.outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
    turningConfig.closedLoop.positionWrappingEnabled(True)
    turningConfig.closedLoop.positionWrappingInputRange(0, ModuleConstants.kTurningEncoderPositionFactor)
    return turningConfig


class ModuleConstants:
    kDrivingMotorIsTalon = False

    # WATCH OUT:
    #  - one or both of two constants below need to be flipped from True to False (by trial and error)
    #  , depending which swerve module you have (MK4i, MK4n, Rev, WCP, ThriftyBot, etc)
    # DC CA 2-18-26 for 2025 robot try turning encoder inverted True (was False)
    kTurningEncoderInverted = False
    kTurningMotorInverted = True

    # The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    # This changes the drive speed of the module (a pinion gear with more teeth will result in a
    # robot that drives faster).
    kDrivingMotorPinionTeeth = 13    # DC CA was 14

    # Calculations required for driving motor conversion factors and feed forward
    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
    kWheelDiameterMeters = 0.0762
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    # 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (
        kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (
        kWheelDiameterMeters * math.pi
    ) / kDrivingMotorReduction  # meters
    kDrivingEncoderVelocityFactor = (
        (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction
    ) / 60.0  # meters per second

    kTurningEncoderPositionFactor = math.tau  # radian
    kTurningEncoderVelocityFactor = math.tau / 60.0  # radians per second

    kTurningEncoderPositionPIDMinInput = 0  # radian
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor  # radian

    # DC CA update pid values to 2025 robot values
    kDrivingP = 0.2
    kDrivingI = 0
    kDrivingD = 0.02
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    kTurningP = .2  # can be dialed down if you see oscillations in the turning motor
    kTurningI = 0
    kTurningD = 0.02
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    kDrivingMotorIdleMode = SparkBase.IdleMode.kBrake
    kTurningMotorIdleMode = SparkBase.IdleMode.kBrake

    kDrivingMotorCurrentLimit = 50  # amp
    kTurningMotorCurrentLimit = 20  # amp

    kDrivingMinSpeedMetersPerSecond = 0.01


class OIConstants:
    kDriverControllerPort = 0
    kDriveDeadband = 0.05


class AutoConstants:
    kUseSqrtControl = True  # improves arrival time and precision for simple driving commands

    # below are really trajectory constants
    kMaxSpeedMetersPerSecond = 3
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularSpeedRadiansPerSecond = math.pi
    kMaxAngularSpeedRadiansPerSecondSquared = math.pi

    kPXController = 1
    kPYController = 1
    kPThetaController = 1     # DC CA was 0.67

    # Constraint for the motion profiled robot angle controller
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
    )
    
class FuelConstants:
    # Motor controller IDs for Fuel Mechanism motors
    FEEDER_MOTOR_ID = 6
    INTAKE_LAUNCHER_MOTOR_ID = 5

    # Current limit and nominal voltage for fuel mechanism motors.
    FEEDER_MOTOR_CURRENT_LIMIT = 60
    LAUNCHER_MOTOR_CURRENT_LIMIT = 60

    # Voltage values for various fuel operations. These values may need to be tuned
    # based on exact robot construction.
    # See the Software Guide for tuning information
    INTAKING_FEEDER_VOLTAGE = -11.0
    INTAKING_INTAKE_VOLTAGE = 9.0
    LAUNCHING_FEEDER_VOLTAGE = 9.0
    LAUNCHING_LAUNCHER_VOLTAGE = 10.6
    SPIN_UP_FEEDER_VOLTAGE = -6.0
    SPIN_UP_SECONDS = 1.0
    
class OperatorConstants:
    # Port constants for driver and operator controllers. These should match the
    # values in the Joystick tab of the Driver Station software
    DRIVER_CONTROLLER_PORT = 0
    OPERATOR_CONTROLLER_PORT = 1

    # This value is multiplied by the joystick value when rotating the robot to
    # help avoid turning too fast and being difficult to control
    DRIVE_SCALING = 0.7
    ROTATION_SCALING = 0.4
