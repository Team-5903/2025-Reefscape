// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.Util;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_MOTOR, MotorType.kBrushless);
  private final PWM linearActuatorLeft = new PWM(Constants.ClimberConstants.LINEAR_ACTUATOR_LEFT);
  private final PWM linearActuatorRight = new PWM(Constants.ClimberConstants.LINEAR_ACTUATOR_RIGHT);
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    linearActuatorLeft.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    linearActuatorRight.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake)
        .softLimit.reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(Constants.ClimberConstants.FORWARD_LIMIT)
        .forwardSoftLimitEnabled(true);
  }

  @Override
  public void periodic() {
    Util.LogSpark("Climber/Motor", climberMotor);
    Logger.recordOutput("Climber/actuatorLeftSet", linearActuatorLeft.getSpeed());
    Logger.recordOutput("Climber/actuatorRightSet", linearActuatorRight.getSpeed());
  }

  // public Command
}
