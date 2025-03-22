// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Util.Util;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMotorLeft = new SparkMax(Constants.IntakeConstants.INTAKE_MOTOR_LEFT, MotorType.kBrushless);
  private final SparkMax intakeMotorRight = new SparkMax(Constants.IntakeConstants.INTAKE_MOTOR_RIGHT, MotorType.kBrushless);
  private final AnalogInput coralStagingBeamBreak = new AnalogInput(Constants.IntakeConstants.CORAL_STAGING_BEAM_BREAK_CHANNEL);
  private final AnalogInput coralBeamBreak = new AnalogInput(Constants.IntakeConstants.CORAL_BEAM_BREAK_CHANNEL);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    
    SparkMaxConfig configLeft = new SparkMaxConfig();
    configLeft
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kCoast)
        .closedLoop.velocityFF(Constants.IntakeConstants.INTAKE_FEED_FORWARD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    configLeft
        .encoder.velocityConversionFactor(Constants.IntakeConstants.INTAKE_CONVERSION_FACTOR);
      
    SparkMaxConfig configRight = new SparkMaxConfig();
      configRight.apply(configLeft)
      .inverted(true);

    intakeMotorLeft.configure(configLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotorRight.configure(configRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    coralStagingBeamBreak.setAverageBits(24);
    coralBeamBreak.setAverageBits(24);
    AnalogInput.setGlobalSampleRate(1500);
  }

  @Override
  public void periodic() {
    Util.LogSpark("Intake/MotorLeft", intakeMotorLeft);
    Util.LogSpark("Intake/MotorRight", intakeMotorRight);
    Logger.recordOutput("Intake/IsCoralStaged", IsCoralStaged());
    Logger.recordOutput("Intake/CoralStagedBeamBreakValue", coralStagingBeamBreak.getAverageVoltage());
    Logger.recordOutput("Intake/IsCoralPresent", IsCoralPresent());
    Logger.recordOutput("Intake/CoralBeamBreakValue", coralBeamBreak.getAverageVoltage());
  }

  public Command RunAtVelocity(double velocity)
  {
    return new InstantCommand(() -> {
      this.RunAtVelocityRaw(velocity);
    });
  }

  public void RunAtVelocityRaw(double velocity)
  {
    RunAtVelocityRaw(velocity, velocity);
  }

  public void RunAtVelocityRaw(double leftVelocity, double rightVelocity)
  {
    intakeMotorLeft.getClosedLoopController().setReference(leftVelocity, ControlType.kVelocity);
    intakeMotorRight.getClosedLoopController().setReference(rightVelocity, ControlType.kVelocity);
  }

  public Command Stop()
  {
    return new InstantCommand(() -> {
      intakeMotorLeft.stopMotor();
      intakeMotorRight.stopMotor();
    });
  }

  public boolean IsCoralPresent()
  {
    return coralBeamBreak.getAverageVoltage() > Constants.IntakeConstants.CORAL_BEAM_BREAK_VOLTAGE_THRESHOLD;
  }

  public boolean IsCoralStaged()
  {
    return coralStagingBeamBreak.getAverageVoltage() > Constants.IntakeConstants.CORAL_BEAM_BREAK_VOLTAGE_THRESHOLD;
  }

  public Command OuttakeL1()
  {
    return RunAtVelocity(1)
      .until(() -> !IsCoralStaged())
      .andThen(new InstantCommand(() -> RunAtVelocityRaw(1.0, 3)))
      // .until(() -> !IsCoralPresent())
      .andThen(new WaitCommand(2))
      .andThen(this.Stop());
  }
}
