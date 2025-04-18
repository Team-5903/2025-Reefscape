// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Volts;

import java.lang.StackWalker.Option;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .softLimit.reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(Constants.ElevatorConstants.FORWARD_LIMIT)
        .forwardSoftLimitEnabled(true);
    configLeft.closedLoop
      .outputRange(Constants.ElevatorConstants.DOWN_SLOW, 1)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(Constants.ElevatorConstants.PID_P);
    configLeft.encoder.positionConversionFactor(Constants.ElevatorConstants.CONVERSION_RATIO);
    // configLeft.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    

    SparkMaxConfig configRight = new SparkMaxConfig();
    configRight
      .apply(configLeft)
      .follow(liftMotorLeft, true);

    // Persist parameters to retain configuration in the event of a power cycle
    liftMotorLeft.configure(configLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    liftMotorRight.configure(configRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    liftMotorLeft.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    Util.LogSpark("Elevator/LeftLiftMotor", liftMotorLeft);
    Util.LogSpark("Elevator/RightLiftMotor", liftMotorRight);
    
    Logger.recordOutput("Elevator/SetpointPosition", getSetpointPosition().name());

    double position = liftMotorLeft.getEncoder().getPosition();
    double minOutput = liftMotorLeft.configAccessor.closedLoop.getMinOutput();

    if(position > ElevatorPosition.L2.height + Constants.ElevatorConstants.DOWN_HYSTERESIS && 
      Math.abs(minOutput - Constants.ElevatorConstants.DOWN_SLOW) < 0.001)
    {
      
      SparkMaxConfig config = new SparkMaxConfig();
      config
        .closedLoop.minOutput(Constants.ElevatorConstants.DOWN_FAST);

      liftMotorLeft.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      liftMotorRight.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    if(position <= ElevatorPosition.L2.height - Constants.ElevatorConstants.DOWN_HYSTERESIS && 
      Math.abs(minOutput - Constants.ElevatorConstants.DOWN_FAST) < 0.001)
    {
      
      SparkMaxConfig config = new SparkMaxConfig();
      config
        .closedLoop.minOutput(Constants.ElevatorConstants.DOWN_SLOW);

      liftMotorLeft.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      liftMotorRight.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

  }

  public ElevatorPosition getSetpointPosition() {
    return currentPosition;
  }

  public void setSetpointPosition(ElevatorPosition currentPosition) {
    this.currentPosition = currentPosition;
    liftMotorLeft.getClosedLoopController().setReference(currentPosition.height, ControlType.kPosition, ClosedLoopSlot.kSlot0, Constants.ElevatorConstants.FEED_FORWARD * Volts.of(12).magnitude());
  }

  public Command setSetpointPositionCommand(ElevatorPosition currentPosition) {
    return new InstantCommand(() -> setSetpointPosition(currentPosition));
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

  public Command DriveManual(DoubleSupplier supplier)
  {
    return new RunCommand(() -> {
      liftMotorLeft.set(supplier.getAsDouble());
    }, this);
  }

  public enum ElevatorPosition {

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
