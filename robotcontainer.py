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
# DC CA gotopoint
from commands.gotopoint import GoToPoint
from commands.trajectory import SwerveTrajectory, JerkyTrajectory
# from commands.autoblueleft import AutoBlueLeft
from constants import AutoConstants, DriveConstants, OIConstants
from subsystems.drivesubsystem import DriveSubsystem, BadSimPhysics
from subsystems.limelight_camera import LimelightCamera
from subsystems.limelight_localizer import LimelightLocalizer

from commands.reset_xy import ResetXY, ResetSwerveFront

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
    # DC CA add limelight camera
        self.camera = LimelightCamera("limelight-pickup")
        # CA DC subsystem from KitBot for intake & launcher
        self.fuelSubsystem = CANFuelSubsystem()

        # autoBlueLeft =  AutoBlueLeft(self.fuelSubsystem)

        # self.drivetrain = Drivetrain()
        # self.driveForwardCommand = DriveForward(self.drivetrain)

        #self.frontCamera = LimelightCamera("limelight-front")

        #self.limelightLocalizer.addCamera(
        #    self.frontCamera,
        #    cameraPoseOnRobot=Translation3d(x=0.40, y=-0.15, z=0.5),
        #    cameraHeadingOnRobot=Rotation2d.fromDegrees(0.0))


        # The driver's controller (joystick)
        self.driverController = CommandGenericHID(OIConstants.kDriverControllerPort)

        # Configure the button bindings and autos
        self.configureButtonBindings()
        self.configureAutos()

        # Configure default command for driving using joystick sticks
        from commands.holonomicdrive import HolonomicDrive
        self.robotDrive.setDefaultCommand(
            HolonomicDrive(
                self.robotDrive,
                forwardSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftY),
                leftSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftX),
                rotationSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kRightX),
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

        # self.driverController.leftBumper().whileTrue(Intake(self.fuelSubsystem))
        # While the right bumper on the operator controller is held, spin up for 1
        # second, then launch fuel. When the button is released, stop.
        # self.driverController.rightBumper().whileTrue(LaunchSequence(self.fuelSubsystem)

        # DC CA set Y to eject fueld
        # DC CA disabled the Eject of the fuel via B so limelight camera can do something!
        # While the A, changed to B button -  is held on the operator controller, eject fuel back out
        # the intake
        yButton = self.driverController.button(XboxController.Button.kY)
        yButton.whileTrue(Eject(self.fuelSubsystem))

       # def turn_to_object():
       #     x = self.camera.getX()
       #     print(f"x={x}")
       #     turn_speed = -0.005 * x
       #     self.robotDrive.rotate(turn_speed)
       #     # if you want your robot to slowly chase that object... replace this line above with: self.robotDrive.arcadeDrive(0.1, turn_speed)

       # bButton = self.driverController.button(XboxController.Button.kB)
       # bButton.whileTrue(commands2.RunCommand(turn_to_object, self.robotDrive))
       # bButton.onFalse(commands2.InstantCommand(lambda: self.robotDrive.drive(0, 0, 0, False, False)))

        # ORIG cmd from KitBot self.driverController.b().whileTrue(Eject(self.fuelSubsystem))
        # DC CA end of copied in code from KitBot for intake & launcher  

        # DC CA below for reference on how buttons are defined and used in CRS project
        # yButton = self.driverController.button(XboxController.Button.kY)
        # yButton.whileTrue(runArm1Command)
        # yButton.whileFalse(stopArm1Command)

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

        # DC CA run arm on button Y held - disabled for KitBot
        # runArm1Command = RunCommand(self.robotDrive.runarm1,self.robotDrive)
        # stopArm1Command = RunCommand(self.robotDrive.stoparm1, self.robotDrive)

        # yButton = self.driverController.button(XboxController.Button.kY)
        # yButton.whileTrue(runArm1Command)
        # yButton.whileFalse(stopArm1Command)

        # DC CA disable original POV buttons and replace with simple slow drive
        # DC CA POV-up forward slow
        POV_driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=0.5, rot=0.0),
                                                self.robotDrive)
        POV_driveBackward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=-0.3, rot=0.0),
                                                 self.robotDrive)
        POV_turnRight = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=0.0, rot=0.3),
                                                self.robotDrive)
        POV_turnLeft = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=0.0, rot=-0.3),
                                                self.robotDrive)
        POV_stop = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=0.0, rot=0.0),
                                                self.robotDrive)
        # DC CA next 2 commands - move left/right based on robot heading
        slideLeft = commands2.RunCommand(lambda: self.robotDrive.drive(xSpeed=0.0, ySpeed=0.3, rotSpeed=0.0, fieldRelative= False, rateLimit=False, square=False ),
                                            self.robotDrive)
        slideRight = commands2.RunCommand(
            lambda: self.robotDrive.drive(xSpeed=0.0, ySpeed=-0.3, rotSpeed=0.0, fieldRelative=False, rateLimit=False,
                                          square=False),
            self.robotDrive)

        povUpButton = self.driverController.povUp()
        povUpButton.whileTrue(POV_driveForward)
        # povUpButton.whileFalse(POV_stop)

        povDownButton = self.driverController.povDown()
        povDownButton.whileTrue(POV_driveBackward)
        # povDownButton.whileFalse(POV_stop)

        povRightButton = self.driverController.povRight()
        povRightButton.whileTrue(slideRight)
        # povRightButton.whileFalse(POV_stop)

        povLeftButton = self.driverController.povLeft()
        povLeftButton.whileTrue(slideLeft)
        # povLeftButton.whileFalse(POV_stop)

        # example 2: when "POV-up" button pressed, reset robot field position to "facing North"
        # resetFacingNorthCommand = ResetXY(x=1.0, y=4.0, headingDegrees=0, drivetrain=self.robotDrive)
        # povUpButton = self.driverController.povUp()
        # povUpButton.whileTrue(resetFacingNorthCommand)

        # example 3: when "POV-down" is pressed, reset robot field position to "facing South"
        # resetFacingSouthCommand = ResetXY(x=7.0, y=4.0, headingDegrees=180, drivetrain=self.robotDrive)
        # povDownButton = self.driverController.povDown()
        # povDownButton.whileTrue(resetFacingSouthCommand)

        # DC CA go to shooting location facing HUB when B is pushed
        # example 4: robot drives this trajectory command when "A" button is pressed
        trajectoryCommand1 = SwerveTrajectory(
            drivetrain=self.robotDrive,
            speed=+1.0,
            waypoints=[
                (3.0, 4.0, 0),  # start at left feeding station: x=1.0, y=7.0, heading=-54 degrees
              #  (1.5, 6.5, 0),  # next waypoint
              #  (2.0, 4.5, 0),  # next waypoint
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
                #  (1.0, 7.0, -54),  # start at left feeding station: x=1.0, y=7.0, heading=-54 degrees
                #  (1.5, 6.5, 0),  # next waypoint
                #  (2.0, 4.5, 0),  # next waypoint
            ],
            endpoint=(0.47, 0.71, 0),  # end point at the reef facing North
            flipIfRed=False,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
        )
        xButton = self.driverController.button(XboxController.Button.kX)
        xButton.whileTrue(trajectoryCommand2)  # while "B" button is pressed, keep running trajectoryCommand1

        # example 5: and when "B" button is pressed, drive the reversed trajectory
        # reversedTrajectoryCommand1 = trajectoryCommand1.reversed()
        # bButton = self.driverController.button(XboxController.Button.kB)
        # bButton.whileTrue(reversedTrajectoryCommand1)  # while "B" button is pressed, keep running this command
        # DC CA disabled B function for CRS so KitBot eject happens

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
        autoblueleftCommand = RunCommand(AutoBlueLeft, self.fuelSubsystem)
        self.chosenAuto = wpilib.SendableChooser()
        # you can also set the default option, if needed
        self.chosenAuto.setDefaultOption("trajectory example", self.getAutonomousTrajectoryExample)
        self.chosenAuto.addOption("left blue", self.getAutonomousLeftBlue)
        self.chosenAuto.addOption("left red", self.getAutonomousLeftRed)
        self.chosenAuto.addOption("AutoBlueLeft", self.CA_AutoBlueLeft )
        self.chosenAuto.addOption("AutoBlueCenter", self.CA_AutoBlueCenter)
        self.chosenAuto.addOption("GetTestCommand", self.getTestCommand )
        self.chosenAuto.addOption("CA Test Auto", self.CA_TestAuto)
        wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)

    def getAutonomousLeftBlue(self):
        setStartPose = ResetXY(x=0.783, y=6.686, headingDegrees=+60, drivetrain=self.robotDrive)
        driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0), self.robotDrive)
        stop = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = setStartPose.andThen(driveForward.withTimeout(1.0)).andThen(stop)
        return command

    def getAutonomousLeftRed(self):
        setStartPose = ResetXY(x=15.777, y=4.431, headingDegrees=-120, drivetrain=self.robotDrive)
        driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0), self.robotDrive)
        stop = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = setStartPose.andThen(driveForward.withTimeout(2.0)).andThen(stop)
        return command

    def getAutonomousTrajectoryExample(self) -> commands2.Command:
        command = SwerveTrajectory(
            drivetrain=self.robotDrive,
            speed=+.5,
            waypoints=[
                (0.25, 0.25, 45.0),  # start at x=1.0, y=4.0, heading=0 degrees (North)
                (0.5, 0.25, 0.0),  # next waypoint: x=2.5, y=5.0
                (0.5, 0.75, 0.0),  # next waypoint
                (0.75, 1.0, -90),  # next waypoint
            ],
            endpoint=(1.25, 1.25, -180),  # end point: x=6.0, y=4.0, heading=180 degrees (South)
            flipIfRed=False,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
         )

        return command

          # commanda0 = InstantCommand(print(f"Step 1"))
          # commanda1 = InstantCommand(SwerveTrajectory(
         # commanda2 = InstantCommand(print(f"Step 2"))
         # commanda3 =  RunCommand(self.robotDrive.runarm1,self.robotDrive)
         # commanda4 =  RunCommand(self.robotDrive.stoparm1,self.robotDrive)

        # -> commands2.Command: (commanda0.andThen(commanda1).andThen(commanda2).andThen(commanda3).andThen(commanda4))



    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode ("test dance") to exercise all subsystems
        """

        # example commands that test drivetrain's motors and gyro (our only subsystem)
        turnRight = AimToDirection(degrees=-45, drivetrain=self.robotDrive, speed=0.25)
        turnLeft = AimToDirection(degrees=45, drivetrain=self.robotDrive, speed=0.25)
        backToZero = AimToDirection(degrees=0, drivetrain=self.robotDrive, speed=0.0)
        backToZero2 = AimToDirection(degrees=0, drivetrain=self.robotDrive, speed=0.0)

        CA_gotopointXY = GoToPoint(3.0,3.0, drivetrain=self.robotDrive, speed=1.0 )
        CA_gotopointXY2 = GoToPoint(3.0, 4.0, drivetrain=self.robotDrive, speed=1.0)
      #  command = turnRight.andThen(turnLeft).andThen(backToZero)
        command = (CA_gotopointXY
                   .andThen(backToZero)
                   .andThen(CA_gotopointXY2)
                   .andThen(backToZero2))

        return command

    # DC CA start of Autonomous - blue/red - left/center/right

    def CA_AutoBlueLeft(self) -> commands2.Command:

        setStartPose = ResetXY(x=0.0, y=0, headingDegrees=180, drivetrain=self.robotDrive)
        C20_driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0),
                                                self.robotDrive)
        C21_driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0),
                                                self.robotDrive)

        C30_driveBackward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=-1.0, rot=0.0), self.robotDrive)
        C40_turnRight = AimToDirection(degrees=-90, drivetrain=self.robotDrive, speed=0.25)
        C50_turnLeft = AimToDirection(degrees=90, drivetrain=self.robotDrive, speed=0.25)
        C60_turnLeft = AimToDirection(degrees=90, drivetrain=self.robotDrive, speed=0.25)
        C70_driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0),
                                                self.robotDrive)

        stop_ABL = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = (setStartPose
                   .andThen(C20_driveForward.withTimeout(2.0))
                   .andThen(C50_turnLeft)
                   .andThen(C21_driveForward.withTimeout(1.5))
                   .andThen(C60_turnLeft)
                   .andThen(C70_driveForward.withTimeout(1.0))
                   .andThen(LaunchSequence(self.fuelSubsystem))
                   .andThen(C30_driveBackward.withTimeout(1.5))
                   .andThen(stop_ABL))

        return command

    # DC CA auto blue center - use SwerveTrajectory to move before launch

    def CA_AutoBlueCenter(self) -> commands2.Command:

        DoSwerveA = SwerveTrajectory(
            drivetrain=self.robotDrive,
            speed=+.5,
            waypoints=[
                (.25, .25, 45.0),  # start at x=1.0, y=4.0, heading=0 degrees (North)
                (.5, .75, 0.0),  # next waypoint: x=2.5, y=5.0
                (0.75, 1.0, 0.0),  # next waypoint
                (1.25, 1.25, -90),  # next waypoint   DC CA cut down x,y for testing
            ],
            endpoint=(2.0, 1.5, -180),  # end point: x=6.0, y=4.0, heading=180 degrees (South)
            flipIfRed=False,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
        )

        setStartPose = ResetXY(x=0.0, y=0, headingDegrees=180, drivetrain=self.robotDrive)
        C20_driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0),
                                                self.robotDrive)
        C21_driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0),
                                                self.robotDrive)

        C30_driveBackward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=-1.0, rot=0.0), self.robotDrive)
        C40_turnRight = AimToDirection(degrees=-90, drivetrain=self.robotDrive, speed=0.25)
        C50_turnLeft = AimToDirection(degrees=90, drivetrain=self.robotDrive, speed=0.25)
        C60_turnLeft = AimToDirection(degrees=90, drivetrain=self.robotDrive, speed=0.25)
        C70_driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0),
                                                self.robotDrive)

        stop_ABL = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))
        stopLaunch = commands2.RunCommand(lambda: LaunchStop)

        command = (setStartPose
                   .andThen(DoSwerveA.withTimeout(3.0))
                   .andThen(LaunchSequence(self.fuelSubsystem).withTimeout(2.0))
                   .andThen(stopLaunch)
                   .andThen(C30_driveBackward.withTimeout(1.5))
                   .andThen(stop_ABL))

        return command

        # DC CA test auto  - use SwerveTrajectory to move before launch

    def CA_TestAutoOrig(self) -> commands2.Command:

        setStartPoseT = ResetXY(x=3.6, y=7.6, headingDegrees=180, drivetrain=self.robotDrive)

        GoToPointA1 = GoToPoint(2,7.0,drivetrain=self.robotDrive, speed=1)
      #  GoToPointA2 = GoToPoint(2.4, 4.0, drivetrain=self.robotDrive, speed=1, finishDirection=Rotation2d.fromDegrees(0))

        DoSwerveAt1 = SwerveTrajectory(
            drivetrain=self.robotDrive, speed=+1.0, waypoints=[],
            endpoint=(2.5, 3.5, 0), stopAtEnd=False, # end point: x, y, heading  at end = keep going
            )

        DoSwerveAt2 = JerkyTrajectory(
            drivetrain=self.robotDrive, speed=+1.0,  waypoints=[],
            endpoint=(2.1, 7.4, 270),  stopAtEnd=False  # end point: x, y, heading  at end = keep going
            )

        DoSwerveAt3 = JerkyTrajectory(
            drivetrain=self.robotDrive, speed=+1.0, waypoints=[],
            endpoint=(2.1, 4.2, 0), stopAtEnd=False,   # end point: x, y, heading  at end = keep going
            )

        C20_driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0),
                                                self.robotDrive)
        C30_driveBackward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=-1.0, rot=0.0),
                                                 self.robotDrive)

        C99_turnNorth = AimToDirection(degrees=0, drivetrain=self.robotDrive, speed=1.0)

        stop_ABLt = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))
        sleepA1 = commands2.RunCommand(lambda: time.sleep(1))
        printA1 = commands2.RunCommand(lambda: print(f"show message"))

        command = (setStartPoseT
               #     .andThen(GoToPointA1.withTimeout(3.0))
               #    .andThen(GoToPointA2.withTimeout(7.0))
                   .andThen(DoSwerveAt2.withTimeout(7.0))
                   .andThen(DoSwerveAt3.withTimeout(7.0))
               #    .andThen(C99_turnNorth.withTimeout(15.0))
               #    .andThen(DoSwerveAt1.withTimeout(9.0))
               #   .andThen(sleepA1.withTimeout(0.1))
               #    .andThen(printA1.withTimeout(.1))
               #    .andThen(LaunchSequence(self.fuelSubsystem).withTimeout(3.0))
               #    .andThen(stopLaunch)
                   .andThen(stop_ABLt))

        return command

    def CA_TestAuto(self) -> commands2.Command:
        TA01_setStartPose = ResetXY(x=3.6, y=7.6, headingDegrees=180, drivetrain=self.robotDrive)

        TA10_DoSwerveAt2 = JerkyTrajectory(
            drivetrain=self.robotDrive, speed=+1.0, waypoints=[],
            endpoint=(2.1, 7.4, 270), stopAtEnd=False  # end point: x, y, heading  at end = keep going
        )

        TA20_DoSwerveAt3 = JerkyTrajectory(
           drivetrain=self.robotDrive, speed=+1.0, waypoints=[],
           endpoint=(2.1, 4.2, 0), stopAtEnd=False,  # end point: x, y, heading  at end = keep going
        )

        TA40_stopFeeder = commands2.RunCommand(lambda: self.fuelSubsystem.setFeederRoller(0))
        TA50_stopRoller = commands2.RunCommand(lambda: self.fuelSubsystem.setIntakeLauncherRoller(0))


        TA90_stop_AB = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = (TA01_setStartPose.withTimeout(.5)
           .andThen(TA10_DoSwerveAt2.withTimeout(5.0))
           .andThen(TA20_DoSwerveAt3.withTimeout(5.0))
           .andThen(LaunchSequence(self.fuelSubsystem).withTimeout(3.0))
           .andThen(TA40_stopFeeder.withTimeout(1.0))
           .andThen(TA50_stopRoller.withTimeout(1.0))
           .andThen(TA90_stop_AB))

        return command
