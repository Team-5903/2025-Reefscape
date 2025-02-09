// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMotorLeft = new SparkMax(Constants.IntakeConstants.INTAKE_MOTOR_LEFT, MotorType.kBrushless);
  private final SparkMax intakeMotorRight = new SparkMax(Constants.IntakeConstants.INTAKE_MOTOR_RIGHT, MotorType.kBrushless);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    
    SparkMaxConfig configLeft = new SparkMaxConfig();
    configLeft
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kCoast)
        .closedLoop.velocityFF(Constants.IntakeConstants.INTAKE_FEED_FORWARD);

    SparkMaxConfig configRight = new SparkMaxConfig();
      configRight.apply(configLeft)
      .inverted(true);

    intakeMotorLeft.configure(configLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotorRight.configure(configRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
