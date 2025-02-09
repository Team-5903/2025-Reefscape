// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax liftMotorLeft = new SparkMax(Constants.ElevatorConstants.LIFT_MOTOR_LEFT, MotorType.kBrushless);
  private final SparkMax liftMotorRight = new SparkMax(Constants.ElevatorConstants.LIFT_MOTOR_RIGHT, MotorType.kBrushless);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    SparkMaxConfig configLeft = new SparkMaxConfig();
    configLeft
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake)
        .softLimit.reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(Constants.ElevatorConstants.FORWARD_LIMIT)
        .forwardSoftLimitEnabled(true);

    SparkMaxConfig configRight = new SparkMaxConfig();
    configRight
      .follow(liftMotorLeft, true);

    // Persist parameters to retain configuration in the event of a power cycle
    liftMotorLeft.configure(configLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    liftMotorRight.configure(configRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
