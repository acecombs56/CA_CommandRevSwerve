from __future__ import annotations
import math

import commands2
import wpimath
import wpilib
import typing
import time

from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import CommandGenericHID
from wpilib import XboxController
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Translation3d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from commands.aimtodirection import AimToDirection
from commands.ca_test_auto import CATestAuto
from commands.ca_far_left import CAFarLeft
from commands.ca_right_center import CARightCenter
from commands.ca_far_left_neutral import CAFarLeftNeutral
from commands.ca_rotate_10_degrees import CARotate10Degrees

# DC CA gotopoint
from commands.gotopoint import GoToPoint
from commands.trajectory import SwerveTrajectory, JerkyTrajectory
# from commands.autoblueleft import AutoBlueLeft
from constants import AutoConstants, DriveConstants, OIConstants
from subsystems.drivesubsystem import DriveSubsystem, BadSimPhysics
from subsystems.limelight_camera import LimelightCamera
from subsystems.limelight_localizer import LimelightLocalizer
from wpilib import SmartDashboard
from commands.reset_xy import ResetXY, ResetSwerveFront
from commands.reset_robot_position import ResetRobotPosition
from commands.reset_robot_positionR import ResetRobotPositionR

from commands.ca_center import CACenter
from commands.ca_center_left import CACenterLeft
from commands.ca_center_right import CACenterRight
from commands.drive_forward import DriveForward


# DC CA copied from KitBot
from constants import OperatorConstants
# from commands.drive import Drive
from commands.eject import Eject
# from commands.exampleauto import ExampleAuto
# from commands.autoblueleft import AutoBlueLeft
from commands.intake import Intake
from commands.launchsequence import LaunchSequence
from commands.launchstop import LaunchStop
# from subsystems.candrivesubsystem import CANDriveSubsystem
from subsystems.canfuelsubsystem import CANFuelSubsystem
from commands.autoblueleft import AutoBlueLeft


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self, robot) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()
        self.limelightLocalizer = LimelightLocalizer(self.robotDrive)
        self.camera = LimelightCamera("limelight")
        self.fuelSubsystem = CANFuelSubsystem()


        self.limelightLocalizer.addCamera(
           self.camera,
           cameraPoseOnRobot=Translation3d(x=0.40, y=-0.15, z=0.5),
           cameraPitchAngleDegrees=10,   # DC CA add as required but need to confirm value
           cameraHeadingOnRobot=Rotation2d.fromDegrees(0.0))


        # The driver's controller (joystick)
        self.driverController = CommandGenericHID(OIConstants.kDriverControllerPort)
        self.operatorController = CommandGenericHID(OIConstants.kOperatorControllerPort)

        # Configure the button bindings and autos
        self.configureButtonBindings()
        self.configureAutos()

        # Configure drive power level chooser on SmartDashboard
        # DC CA powerChooser is a good example of selecting options from
        #   the Smart Dashboard during TeleOp!   Then used in HolonomicDrive default command!
        self.powerChooser = wpilib.SendableChooser()
        self.powerChooser.addOption("25% Power", 0.25)
        self.powerChooser.setDefaultOption("50% Power", 0.50)
        self.powerChooser.addOption("75% Power", 0.75)
        self.powerChooser.addOption("100% Power", 1.0)
        wpilib.SmartDashboard.putData("Drive Power Level", self.powerChooser)

        # Configure POV power level chooser on SmartDashboard
        self.POVpowerChooser = wpilib.SendableChooser()
        self.POVpowerChooser.addOption("25% POV Power", 0.25)
        self.POVpowerChooser.setDefaultOption("50% POV Power", 0.50)
        self.POVpowerChooser.addOption("75% POV Power", 0.75)
        self.POVpowerChooser.addOption("100% POV Power", 1.0)
        wpilib.SmartDashboard.putData("POV Power Level", self.POVpowerChooser)

        # DC CA SmartDashboard chooser for POV slide left/right fieldRelative mode
        self.slideFieldRelativeChooser = wpilib.SendableChooser()
        self.slideFieldRelativeChooser.setDefaultOption("False (robot-relative)", False)
        self.slideFieldRelativeChooser.addOption("True (field-relative)", True)
        wpilib.SmartDashboard.putData("Slide fieldRelative Mode", self.slideFieldRelativeChooser)

        # Configure default command for driving using joystick sticks
        from commands.holonomicdrive import HolonomicDrive
        self.robotDrive.setDefaultCommand(
            HolonomicDrive(
                self.robotDrive,
                # forwardSpeed=lambda: self.driverController.getRawAxis(XboxController.Axis.kLeftY),
                # DC CA power chooser for "forward" speed is an example
                forwardSpeed=lambda: self.driverController.getRawAxis(XboxController.Axis.kLeftY) * (
                        self.powerChooser.getSelected() or 0.50),
                leftSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftX) * (
                        self.powerChooser.getSelected() or 0.50),
                rotationSpeed=lambda: self.driverController.getRawAxis(XboxController.Axis.kRightX),
                deadband=OIConstants.kDriveDeadband,
                fieldRelative=True,
                rateLimit=True,
                square=True,
            )
        )

        if commands2.TimedCommandRobot.isSimulation():
            self.robotDrive.simPhysics = BadSimPhysics(self.robotDrive, robot)


    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        # DC CA copied in KitBot controller bindings for intake & launcher 
        # While the left bumper on operator controller is held, intake Fuel
        # DC CA define left and right bumpers per CRS project
        rbButton = self.driverController.button(XboxController.Button.kRightBumper)
        lbButton = self.driverController.button(XboxController.Button.kLeftBumper)
        lbButton.whileTrue(Intake(self.fuelSubsystem))
        rbButton.whileTrue(LaunchSequence(self.fuelSubsystem))

        # DC CA set Y to eject fuel
        yButton = self.driverController.button(XboxController.Button.kY)
        yButton.whileTrue(Eject(self.fuelSubsystem))

        # aButton = self.driverController.button(XboxController.Button.kA)
        # aButton.whileTrue(commands2.RunCommand(self.turn_to_object, self.robotDrive))
        # aButton.onFalse(commands2.InstantCommand(lambda: self.robotDrive.drive(0, 0, 0, False, False)))

        # DC CA lock wheels when A is pressed so robot cannot be pushed
        # example 1: hold the wheels in "swerve X brake" position, when "X" button is pressed
        brakeCommand = RunCommand(self.robotDrive.setX, self.robotDrive)
        aButton = self.driverController.button(XboxController.Button.kA)
        aButton.whileTrue(brakeCommand)  # while "X" button is True (pressed), keep executing the brakeCommand

        # DC CA define Xbox buttons:
        #  example of getting a trigger value:
        #  tVal = self.driverController.XboxController.getRightTriggerAxis()
        # note the trigger returns a value that can be compared and a function executed!
        # button(XboxController.Button.k)   # examples - see below
        #  .popUp .popUpLeft .povLeft .popDownLeft .popDown
        #  .pov.DownRight .povRight .povUpRight
        #  xButton = self.driverController.button(XboxController.Button.kX)
        #   .kY .kB . kA  .kLeftStick .kLeftBumper .kRightStick .kRightBumper

        # DC CA disable original POV buttons and replace with simple slow drive
        # DC CA POV-up forward slow
        # POV_driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(
        #    xSpeed= (1 * (self.POVpowerChooser.getSelected() or 0.50)), rot=0.0),  self.robotDrive)
        # POV_driveBackward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(
        #    xSpeed=(-1 * (self.POVpowerChooser.getSelected() or 0.50)), rot=0.0), self.robotDrive)
        POV_turnRight = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=0.0, rot=0.3),
                                                self.robotDrive)
        POV_turnLeft = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=0.0, rot=-0.3),
                                                self.robotDrive)
        POV_stop = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=0.0, rot=0.0),
                                                self.robotDrive)

        # DC CA next 2 commands - move fwd/back, fieldRelative set via SmartDashboard chooser
        POV_driveForward = commands2.RunCommand(
            lambda: self.robotDrive.drive(
                xSpeed=-1.0 * (self.POVpowerChooser.getSelected() or 0.50),
                ySpeed=0.0,
                rotSpeed=0.0,
                fieldRelative=self.slideFieldRelativeChooser.getSelected(),
                rateLimit=False, square=False),
            self.robotDrive)

        POV_driveBackward = commands2.RunCommand(
                lambda: self.robotDrive.drive(
                    xSpeed=1.0 * (self.POVpowerChooser.getSelected() or 0.50),
                    ySpeed=0.0,
                    rotSpeed=0.0,
                    fieldRelative=self.slideFieldRelativeChooser.getSelected(),
                    rateLimit=False, square=False),
                   self.robotDrive)

        # DC CA next 2 commands - move left/right, fieldRelative set via SmartDashboard chooser
        slideLeft = commands2.RunCommand(
            lambda: self.robotDrive.drive(xSpeed=0.0,
                    ySpeed=1.0 * (self.POVpowerChooser.getSelected() or 0.50),
                    rotSpeed=0.0,
                    fieldRelative=self.slideFieldRelativeChooser.getSelected(),
                    rateLimit=False, square=False),
            self.robotDrive)

        slideRight = commands2.RunCommand(
            lambda: self.robotDrive.drive(xSpeed=0.0,
                    ySpeed=-1.0 * (self.POVpowerChooser.getSelected() or 0.50),
                    rotSpeed=0.0,
                    fieldRelative=self.slideFieldRelativeChooser.getSelected(),
                    rateLimit=False, square=False),
            self.robotDrive)


        povUpButton = self.driverController.povUp()
        #povUpButton.whileTrue(POV_driveForward)
        povUpButton.whileTrue(POV_driveForward)
        # povUpButton.whileTrue(slideLeft)
        # povUpButton.whileFalse(POV_stop)

        povDownButton = self.driverController.povDown()
        povDownButton.whileTrue(POV_driveBackward)
        # povDownButton.whileTrue(slideRight)
        # povDownButton.whileFalse(POV_stop)

        povRightButton = self.driverController.povRight()
        povRightButton.whileTrue(slideRight)
        # povRightButton.whileTrue(POV_driveBackward)
        # povRightButton.whileFalse(POV_stop)

        povLeftButton = self.driverController.povLeft()
        povLeftButton.whileTrue(slideLeft)
        # povLeftButton.whileTrue(POV_driveForward)
        # povLeftButton.whileFalse(POV_stop)

        # DC CA go to shooting location facing HUB when B is pushed
        # example 4: robot drives this trajectory command when "A" button is pressed
        trajectoryCommand1 = SwerveTrajectory(
            drivetrain=self.robotDrive,
            speed=+1.0,
            waypoints=[
                (3.0, 4.0, 0),  # start at left feeding station: x=1.0, y=7.0, heading=-54 degrees
                ],
            endpoint=(3.2, 4.0, 0),  # end point at the reef facing North
            flipIfRed=False,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
        )
        bButton = self.driverController.button(XboxController.Button.kB)
        bButton.whileTrue(trajectoryCommand1)  # while "B" button is pressed, keep running trajectoryCommand1

        # DC CA got to Output facing N when X is pressed
        # DC CA go to shooting location facing HUB when B is pushed
        # example 4: robot drives this trajectory command when "A" button is pressed
        trajectoryCommand2 = SwerveTrajectory(
            drivetrain=self.robotDrive,
            speed=+1.0,
            waypoints=[
               ],
            endpoint=(0.47, 0.71, 0),  # end point at the reef facing North
            flipIfRed=False,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
        )
        xButton = self.driverController.button(XboxController.Button.kX)
        xButton.whileTrue(trajectoryCommand2)  # while "B" button is pressed, keep running trajectoryCommand1

       # DC CA fix for rotate until swerve wheels can be fixed/corrected  3/18
        rightStickButton = self.driverController.button(XboxController.Button.kRightStick)
        rightStickButton.whileTrue(commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=0.0, rot=0.60)
                                                        , self.robotDrive))
        # Reset robot position to x=3, y=3, rotation=180 when back button is pressed
        resetPositionCommand = ResetRobotPosition(self.robotDrive)
        backButton = self.driverController.button(XboxController.Button.kBack)
        backButton.onTrue(resetPositionCommand)
        # Reset robot position to x=3, y=3, rotation=0 when back button is pressed
        resetPositionCommandR = ResetRobotPositionR(self.robotDrive)
        startButton = self.driverController.button(XboxController.Button.kStart)
        startButton.onTrue(resetPositionCommandR)

        # DC CA operator controller bindings
        # DC CA Y button: drive forward ~1 meter quickly
        yOButton = self.operatorController.button(XboxController.Button.kY)
        yOButton.whileTrue(DriveForward(distanceMeters=1.0, speed=0.5, drivetrain=self.robotDrive))

        # turn to April Tag that is within 30 degrees
        aOButton = self.operatorController.button(XboxController.Button.kA)
        aOButton.whileTrue(commands2.RunCommand(self.turn_to_object, self.robotDrive))
        aOButton.onFalse(commands2.InstantCommand(lambda: self.robotDrive.drive(0, 0, 0, False, False)))

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous
        """
        command = self.chosenAuto.getSelected()
        return command()

    def configureAutos(self):
        # DC CA - define commands for Autonomous as "commands"
        self.chosenAuto = wpilib.SendableChooser()
        # you can also set the default option, if needed
        # self.chosenAuto.setDefaultOption("CA Test Auto Simple", self.CA_TestAutoSimple)
        #self.chosenAuto.addOption("CA Far Left Auto", self.CA_FarLeftAuto)
        #self.chosenAuto.addOption("CA Far Left Neutral Auto", self.CA_FarLeftNeutralAuto)
        self.chosenAuto.addOption("CA Center Right Auto", self.CA_CenterRightAuto)
        self.chosenAuto.addOption("CA Center Left Auto", self.CA_CenterLeftAuto)
        self.chosenAuto.setDefaultOption("CA Center Auto", self.CA_CenterAuto)

        wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)


    # new method: this shows how to call a command from \Commands
    def CA_TestAutoSimple(self) -> commands2.Command:
        return CATestAuto(self.fuelSubsystem, drivetrain=self.robotDrive)

    def CA_FarLeftAuto(self) -> commands2.Command:
        return CAFarLeft(self.fuelSubsystem, drivetrain=self.robotDrive)

    def CA_FarLeftNeutralAuto(self) -> commands2.Command:
        return CAFarLeftNeutral(self.fuelSubsystem, drivetrain=self.robotDrive)

    def CA_CenterRightAuto(self) -> commands2.Command:
        return CACenterRight(self.fuelSubsystem, drivetrain=self.robotDrive)

    def CA_CenterAuto(self) -> commands2.Command:
        return CACenter(self.fuelSubsystem, drivetrain=self.robotDrive)

    def CA_CenterLeftAuto(self) -> commands2.Command:
        return CACenterLeft(self.fuelSubsystem, drivetrain=self.robotDrive)

    def turn_to_object(self):
         x = self.camera.getX()
         y = self.camera.getY()
         a = self.camera.getA()
         v = self.camera.hasDetection()

         if v is None:
             v = 0

         print(f"x={x}")

         SmartDashboard.putNumber("LL_TX", x)
         SmartDashboard.putNumber("LL_TY", y)
         SmartDashboard.putNumber("LL_TA", a)
         SmartDashboard.putNumber("LL Detection = 1:", v)
         # SmartDashboard.putNumber("LL_TA", limelightTable.getEntry("ta").getDouble(0.0));
         # SmartDashboard.putBoolean("LL_HasTarget", limelightTable.getEntry("tv").getDouble(0.0) == 1.0);
         # SmartDashboard.putNumber("LL_Latency", limelightTable.getEntry("tl").getDouble(0.0));

         turn_speed = -0.01 * x
         self.robotDrive.rotate(turn_speed)
