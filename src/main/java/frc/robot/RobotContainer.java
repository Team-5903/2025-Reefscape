// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.List;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

  private final LoggedDashboardChooser<Command> autoChooser;
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> -driverXbox.getRightX())
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
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    registerAndNameCommand("test", Commands.print("I EXIST"));
    registerAndNameCommand("AutoIntake", new AutoIntakeCommand(intake));
    registerAndNameCommand("IntakeManual", intake.RunAtVelocity(1));
    registerAndNameCommand("IntakeStop", intake.Stop());
    registerAndNameCommand("ElevatorIntake", elevator.setSetpointPositionCommand(ElevatorPosition.INTAKE));
    registerAndNameCommand("ElevatorL1", elevator.setSetpointPositionCommand(ElevatorPosition.L1));
    registerAndNameCommand("ElevatorL2", elevator.setSetpointPositionCommand(ElevatorPosition.L2));
    registerAndNameCommand("ElevatorL3", elevator.setSetpointPositionCommand(ElevatorPosition.L3));
    registerAndNameCommand("ElevatorL4", elevator.setSetpointPositionCommand(ElevatorPosition.L4));
    
    LoggedPowerDistribution.getInstance(Constants.PDH_ID, ModuleType.kRev);
    autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
  }


  public void registerAndNameCommand(String name, Command command)
  {
    NamedCommands.registerCommand(name, command);
    nameCommand(name, NamedCommands.getCommand(name));
  }

  public Command nameCommand(String name, Command command)
  {
    command.setName(name);
    return command;
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

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

  
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // elevator.setDefaultCommand(elevator.DriveManual(() -> operatorXbox.getRightTriggerAxis() - operatorXbox.getLeftTriggerAxis()));

    driverXbox
      .rightBumper()
      .and(() -> !elevator.isAtTop().getAsBoolean())
      .whileTrue(
        elevator.IncreaseHeightSetpoint()
        .andThen(new WaitCommand(0.25))
        .repeatedly()
      );

    driverXbox
      .rightBumper()
      .and(elevator.isAtTop())
      .onTrue(new InstantCommand(() -> driverXbox.setRumble(RumbleType.kRightRumble, 1.0)))
      .onFalse(new InstantCommand(() -> driverXbox.setRumble(RumbleType.kRightRumble, 0.0)));

    driverXbox
      .leftBumper()
      .and(() -> !elevator.isAtBottom().getAsBoolean())
      .whileTrue(
        elevator.DecreaseHeightSetpoint()
        .andThen(new WaitCommand(0.25))
        .repeatedly()
      );

    driverXbox
      .leftBumper()
      .and(elevator.isAtBottom())
      .onTrue(new InstantCommand(() -> driverXbox.setRumble(RumbleType.kLeftRumble, 1.0)))
      .onFalse(new InstantCommand(() -> driverXbox.setRumble(RumbleType.kLeftRumble, 0.0)));

    // driverXbox
    //   .a()
    //   .onTrue(intake.RunAtVelocity(1))
    //   .onFalse(intake.Stop());

    driverXbox
      .a()
      .and(() -> !intake.IsCoralPresent())
      .and(() -> elevator.getSetpointPosition() == ElevatorSubsystem.ElevatorPosition.INTAKE)
      .onTrue(new AutoIntakeCommand(intake)
        .andThen(new InstantCommand(() -> driverXbox.setRumble(RumbleType.kBothRumble, 1.0)))
        .andThen(new WaitCommand(0.25))
        .andThen(new InstantCommand(() -> driverXbox.setRumble(RumbleType.kBothRumble, 0.0)))
      );

      
    driverXbox
      .a()
      .and(() -> elevator.getSetpointPosition() != ElevatorSubsystem.ElevatorPosition.INTAKE)
      .onTrue(intake.RunAtVelocity(1))
      .onFalse(intake.Stop());
    
    driverXbox
      .leftTrigger(0.2)
      .whileTrue(new RunCommand(() -> intake.RunAtVelocityRaw(-driverXbox.getLeftTriggerAxis() * Constants.IntakeConstants.MAX_SPEED), intake))
      .onFalse(intake.Stop());
      
    driverXbox
      .rightTrigger(0.2)
      .whileTrue(new RunCommand(() -> intake.RunAtVelocityRaw(driverXbox.getRightTriggerAxis() * Constants.IntakeConstants.MAX_SPEED), intake))
      .onFalse(intake.Stop());

    driverXbox
      .y()
      .onTrue(intake.RunAtVelocity(-1))
      .onFalse(intake.Stop());


    driverXbox
      .povDown()
      .or(driverXbox.x())
      .onTrue(
        climber
          .DropCoralChute()
          .andThen(climber.RaiseArm())
      );

    driverXbox
      .povUp()
      .onTrue(
        climber
          .LowerArm()
          .andThen(climber.RaiseCoralChute())
      );

    driverXbox
      .povLeft()
      .or(driverXbox.b())
      .and(climber.isChuteOpen())
      .onTrue(climber.ClimbArm());

    driverXbox
      .leftStick()
      .onTrue(new InstantCommand(() -> {
        driveAngularVelocity
          .scaleTranslation(0.4)
          .scaleRotation(0.4);
      }));

    driverXbox
      .rightStick()
      .onTrue(new InstantCommand(() -> {
        driveAngularVelocity
          .scaleTranslation(0.8)
          .scaleRotation(0.8);
      }));


    // if (Robot.isSimulation())
    // {
    //   driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    //   driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    // }
    // if (DriverStation.isTest())
    // {
    //   drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

    //   driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //   driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //   driverXbox.back().whileTrue(drivebase.centerModulesCommand());
    //   driverXbox.leftBumper().onTrue(Commands.none());
    //   driverXbox.rightBumper().onTrue(Commands.none());
    // } else
    // {
    //   driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //   driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    //   driverXbox.b().whileTrue(
    //       drivebase.driveToPose(
    //           new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                           );
    //   driverXbox.y().onTrue(Commands.runOnce(() -> SimulatedArena.getInstance().resetFieldForAuto(), drivebase));
    //   driverXbox.start().whileTrue(Commands.none());
    //   driverXbox.back().whileTrue(Commands.none());
    //   driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //   driverXbox.rightBumper().onTrue(Commands.none());
    // }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.get();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public List<SubsystemBase> getSubsystems(){
    return List.of(drivebase, elevator, climber, intake);
  }
}
