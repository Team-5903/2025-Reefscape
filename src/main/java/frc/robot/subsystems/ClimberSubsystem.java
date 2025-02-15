// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Util.Util;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_MOTOR, MotorType.kBrushless);
  private final PWM linearActuatorLeft = new PWM(Constants.ClimberConstants.LINEAR_ACTUATOR_LEFT);
  private final PWM linearActuatorRight = new PWM(Constants.ClimberConstants.LINEAR_ACTUATOR_RIGHT);
  private boolean chuteOpen = false;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    linearActuatorLeft.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    linearActuatorRight.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(50)
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .softLimit.reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(Constants.ClimberConstants.FORWARD_LIMIT)
        .forwardSoftLimitEnabled(true);
    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(Constants.ClimberConstants.PID_P);

    climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    Util.LogSpark("Climber/Motor", climberMotor);
    Logger.recordOutput("Climber/actuatorLeftSet", linearActuatorLeft.getSpeed());
    Logger.recordOutput("Climber/actuatorRightSet", linearActuatorRight.getSpeed());
  }

  public Command DropCoralChute ()
  {
    return runOnce(() -> {
      linearActuatorLeft.setSpeed(-1);
      linearActuatorRight.setSpeed(-1);
    })
    .andThen(new WaitCommand(5))
    .andThen(
      new InstantCommand(() -> {
        chuteOpen = true;
      })
    );
  }

  public Command RaiseCoralChute()
  {
    return runOnce(() -> {
      chuteOpen = false;
      linearActuatorLeft.setSpeed(1);
      linearActuatorRight.setSpeed(1);
    })
    .andThen(new WaitCommand(5));
  }

  public Command RaiseArm()
  {
    return runOnce(() -> {
      climberMotor.getClosedLoopController().setReference(Constants.ClimberConstants.FORWARD_LIMIT, ControlType.kPosition);
    })
    .andThen(new WaitCommand(5));//TODO: replace with setpoint checking
  }

  public Command LowerArm()
  {
    return runOnce(() -> {
      climberMotor.getClosedLoopController().setReference(0, ControlType.kPosition);
    })
    .andThen(new WaitCommand(5));//TODO: replace with setpoint checking
  }

  public Command ClimbArm()
  {
    return runOnce(() -> {
      climberMotor.getClosedLoopController().setReference(Constants.ClimberConstants.CLIMB_POSITION, ControlType.kPosition);
    })
    .andThen(new WaitCommand(5));//TODO: replace with setpoint checking
  }

  public BooleanSupplier isChuteOpen()
  {
    return () -> chuteOpen;
  }
}
