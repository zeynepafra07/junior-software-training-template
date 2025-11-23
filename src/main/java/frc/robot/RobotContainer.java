// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import frc.robot.commands.CloseClimb;
import frc.robot.commands.OpenClimb;
import frc.robot.commands.getL2ALgae;
import frc.robot.commands.getL3ALgae;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;
import frc.robot.commands.L1Shooting;
import frc.robot.commands.L2Shooting;
import frc.robot.commands.L3Shooting;
import frc.robot.commands.L4Shooting;
import frc.robot.commands.shootAlgaeToBarge;
import frc.robot.commands.shootAlgaetoNet;
import frc.robot.commands.NormalMode;
import frc.robot.commands.FastMode;
import frc.robot.commands.SlowMode;
import frc.robot.commands.ResetEncoder;

import java.io.File;
import swervelib.SwerveInputStream;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
                                                                                private final ClimbSubsystem climb = new ClimbSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final Command closeClimbCommand = new closeClimb(climb);
  private final Command openClimbCommand = new openClimb(climb, elevator);
  private final Command getL2ALgaeCommand = new getL2ALgae(shooter, elevator);
  private final Command getL3ALgaeCommand = new getL3ALgae(shooter, elevator);
  private final Command intakeCommand = new Intake(intake);
  private final Command outtakeCommand = new Outtake(intake);
  private final Command L1ShootCommand = new L1Shooting(shooter, elevator);
  private final Command L2ShootCommand = new L2Shooting(shooter, elevator);
  private final Command L3ShootCommand = new L3Shooting(shooter, elevator);
  private final Command L4ShootCommand = new L4Shooting(shooter, elevator);
  private final Command shootAlgaeBargeCommand = new shootAlgaeToBarge(shooter, elevator);
  private final Command shootAlgaetoNetCommand = new shootAlgaetoNet(shooter, elevator);
  private final Command normalModeCommand = new NormalMode(climb);
  private final Command fastModeCommand = new FastMode(climb);
  private final Command slowModeCommand = new SlowMode(climb);
  private final Command resetEncoderCommand = new ResetEncoder(elevator);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .headingOffset(true)
                                                                               .headingOffset(Rotation2d.fromDegrees(
                                                                                   0));
  



  AutoBuilder.configurePathPlanner(
    (Pose2d targetPose, Command command) -> drivebase.driveToPoseCommand(targetPose),
    (Pose2d initialPose) -> drivebase.resetOdometry(initialPose),
    drivebase::getPose,
    () -> {
      var alliance = DriverStation.getAlliance();
      if(alliance.isPresent())
      {
        return alliance.get() ==  DriverStation.Alliance.Blue;
      }
      return false;
    },
    drivebase
    );

  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    NamedCommands.registerCommand("Intake Command", new Intake(intake));
    NamedCommands.registerCommand("L4 Shoot Command", new L4Shooting(shooter, elevator));

    Command auto1 = AutoBuilder.buildAutoCommand("Blue_1", drivebase, true);
    Command auto2 = AutoBuilder.buildAutoCommand("Blue_2", drivebase, true);
    Command auto3 = AutoBuilder.buildAutoCommand("Blue_3", drivebase, true);
    Command auto4 = AutoBuilder.buildAutoCommand("Red_1", drivebase, true);
    Command auto5 = AutoBuilder.buildAutoCommand("Red_2", drivebase, true);
    Command auto6 = AutoBuilder.buildAutoCommand("Red_3", drivebase, true);

    autoChooser.setDefaultOption("Blue Auto 1", auto1);
    autoChooser.addOption("Blue Auto 2", auto2);
    autoChooser.addOption("Blue Auto 3", auto3);
    autoChooser.addOption("Red Auto 1", auto4);
    autoChooser.addOption("Red Auto 2", auto5);
    autoChooser.addOption("Red Auto 3", auto6);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
     

//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue(shootAlgaeToBargeCommand);
      driverXbox.b().onTrue(getL3ALgaeCommand);
      driverXbox.x().onTrue(getL2ALgaeCommand);
      driverXbox.y().onTrue(shootAlgaetoNetCommand);
      driverXbox.start().whileTrue(fastModeCommand);
      driverXbox.back().whileTrue(slowModeCommand);
      driverXbox.leftBumper().onTrue(L1ShootCommand);
      driverXbox.rightBumper().onTrue(L3ShootCommand);
      driverXbox.rightTrigger().onTrue(L2ShootCommand);
      driverXbox.leftTrigger().onTrue(L4ShootCommand);
      driverXbox.povUp().onTrue(intakeCommand);
      driverXbox.povDown().onTrue(outtakeCommand);
      driverXbox.povRight().onTrue(openClimbCommand);
      driverXbox.povLeft().onTrue(closeClimbCommand);
      driverXbox.leftStick().onTrue(resetEncoderCommand);
      driverXbox.rightStick().onTrue(normalModeCommand);
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
