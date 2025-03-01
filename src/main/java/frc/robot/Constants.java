// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Util.Util;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(19.3);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final int PDH_ID = 14;
//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }
  public static final class ElevatorConstants
  { 
     public static final int LIFT_MOTOR_LEFT = 9;
     public static final int LIFT_MOTOR_RIGHT = 10;

    public static final double FORWARD_LIMIT = 135.5;

    public static final double SPOOL_DIAMETER = Units.inchesToMeters(1.0);
    public static final double GEARBOX_RATIO = 15.0;
    public static final double CONVERSION_RATIO = 1;//(1/GEARBOX_RATIO);//* (SPOOL_DIAMETER * Math.PI);

    public static final double INTAKE_HEIGHT = 0.0;
    public static final double L1_HEIGHT = 10.0;
    public static final double L2_HEIGHT = 30.0;
    public static final double L3_HEIGHT = 65.0;
    public static final double L4_HEIGHT = 130.0;
    
    public static final double SETPOINT_TOLERANCE = 1.0;
    public static final double FEED_FORWARD = 0.02;
    public static final double PID_P = 0.3;

    public static final double DOWN_FAST = -0.8;
    public static final double DOWN_SLOW = -0.4;
    public static final double DOWN_HYSTERESIS = 1.0;

  }
  public static final class IntakeConstants
  { 
     public static final int INTAKE_MOTOR_LEFT = 11;
     public static final int INTAKE_MOTOR_RIGHT = 12;

    public static final double INTAKE_FEED_FORWARD = 0.34;
    public static final int CORAL_STAGING_BEAM_BREAK_CHANNEL = 0;
    public static final int CORAL_BEAM_BREAK_CHANNEL = 1;
    public static final double INTAKE_CONVERSION_FACTOR = ((1.0/20.0) * (Units.inchesToMeters(4) * Math.PI)) / 60.0;
    public static final double CORAL_BEAM_BREAK_VOLTAGE_THRESHOLD = 1.0;
    public static final double MAX_SPEED = 2.0;
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class FieldConstants 
  {
    public static final Translation2d reefCenter = new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
    public static Distance coralStationAutomationZone = Distance.ofBaseUnits(1.0, edu.wpi.first.units.Units.Meters);
    private static final Translation2d m_coralStationLeft = new Translation2d(0.962, 7.472);
    private static final Translation2d m_coralStationRight = Util.mirrorTranslation(m_coralStationLeft);
    public static final Supplier<Translation2d> coralStationLeft = () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? FlippingUtil.flipFieldPosition(m_coralStationLeft) : m_coralStationLeft;
    public static final Supplier<Translation2d> coralStationRight = () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? FlippingUtil.flipFieldPosition(m_coralStationRight) : m_coralStationRight;

    private static final List<Pose2d> m_coralStationPoses = new ArrayList<>() {
      {
        add(new Pose2d(1.61, 7.337, Rotation2d.fromDegrees(-55)));
        add(new Pose2d(0.755, 6.724, Rotation2d.fromDegrees(-55)));
        add(Util.mirrorPose2d(get(0)));
        add(Util.mirrorPose2d(get(1)));
      }
    };

    public static final Supplier<List<Pose2d>> coralStationPoses = () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? m_coralStationPoses.stream().map(FlippingUtil::flipFieldPose).toList() : m_coralStationPoses;
  }

  public static final class ClimberConstants 
  {
    public static final int CLIMBER_MOTOR = 13;
    public static final double FORWARD_LIMIT = 210;
    public static final int LINEAR_ACTUATOR_LEFT = 0;
    public static final int LINEAR_ACTUATOR_RIGHT = 1;
    public static final int CLIMB_POSITION = 70;
    public static final double PID_P = 0.3;
    public static final double ARM_SETPOINT_TOLERANCE = 5;
  }
}
