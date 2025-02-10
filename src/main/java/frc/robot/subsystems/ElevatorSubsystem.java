// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.lang.StackWalker.Option;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.Util;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax liftMotorLeft = new SparkMax(Constants.ElevatorConstants.LIFT_MOTOR_LEFT, MotorType.kBrushless);
  private final SparkMax liftMotorRight = new SparkMax(Constants.ElevatorConstants.LIFT_MOTOR_RIGHT, MotorType.kBrushless);

  private ElevatorPosition currentPosition = ElevatorPosition.INTAKE;

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
    configLeft.encoder.positionConversionFactor(Constants.ElevatorConstants.CONVERSION_RATIO);

    SparkMaxConfig configRight = new SparkMaxConfig();
    configRight
      .apply(configLeft)
      .follow(liftMotorLeft, true);

    // Persist parameters to retain configuration in the event of a power cycle
    liftMotorLeft.configure(configLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    liftMotorRight.configure(configRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    Util.LogSpark("Elevator/LeftLiftMotor", liftMotorLeft);
    Util.LogSpark("Elevator/RightLiftMotor", liftMotorRight);
    
    Logger.recordOutput("Elevator/SetpointPosition", getSetpointPosition().name());
  }

  public ElevatorPosition getSetpointPosition() {
    return currentPosition;
  }

  public void setSetpointPosition(ElevatorPosition currentPosition) {
    this.currentPosition = currentPosition;
    liftMotorLeft.getClosedLoopController().setReference(currentPosition.height, ControlType.kMAXMotionPositionControl);
  }

  public Command IncreaseHeightSetpoint()
  {
    return new InstantCommand(() -> {
      if(getSetpointPosition().nextPosition.isPresent())
      {
        setSetpointPosition(getSetpointPosition().nextPosition.get());
      }
    });
  }

  public Command DecreaseHeightSetpoint()
  {
    return new InstantCommand(() -> {
      if(getSetpointPosition().lastPosition.isPresent())
      {
        setSetpointPosition(getSetpointPosition().lastPosition.get());
      }
    });
  }

  public BooleanSupplier isAtBottom()
  {
    return () -> getSetpointPosition() == ElevatorPosition.INTAKE;
  }

  public BooleanSupplier isAtTop()
  {
    return () -> getSetpointPosition() == ElevatorPosition.L4;
  }

  public BooleanSupplier isAtSetpoint()
  {
    return () -> Math.abs(liftMotorLeft.getEncoder().getPosition() - getSetpointPosition().height) < Constants.ElevatorConstants.SETPOINT_TOLERANCE;
  }

  enum ElevatorPosition {

    INTAKE(Constants.ElevatorConstants.INTAKE_HEIGHT, Optional.empty()),
    L1(Constants.ElevatorConstants.L1_HEIGHT, Optional.of(INTAKE)),
    L2(Constants.ElevatorConstants.L2_HEIGHT, Optional.of(L1)),
    L3(Constants.ElevatorConstants.L3_HEIGHT, Optional.of(L2)),
    L4(Constants.ElevatorConstants.L4_HEIGHT, Optional.of(L3));


    public final double height;
    public final Optional<ElevatorPosition> lastPosition;
    public Optional<ElevatorPosition> nextPosition = Optional.empty();

    ElevatorPosition(double height, Optional<ElevatorPosition> lastPosition)
    {
      this.height = height;
      this.lastPosition = lastPosition;

      if(lastPosition.isPresent())
      {
        lastPosition.get().nextPosition = Optional.of(this);
      }
    }
  }
}
