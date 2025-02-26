package frc.robot.Util;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.FlippingUtil;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Util {

    public static void LogSpark(String logName, SparkBase sparkMax)
    {
        Logger.recordOutput(logName + "/AppliedOutput", sparkMax.getAppliedOutput());
        Logger.recordOutput(logName + "/Velocity", sparkMax.getEncoder().getVelocity());
        Logger.recordOutput(logName + "/Position", sparkMax.getEncoder().getPosition());
        Logger.recordOutput(logName + "/Current", sparkMax.getOutputCurrent());
        Logger.recordOutput(logName + "/Faults", sparkMax.getFaults().rawBits);
        Logger.recordOutput(logName + "/StickyFaults", sparkMax.getStickyFaults().rawBits);
        Logger.recordOutput(logName + "/Temp", sparkMax.getMotorTemperature());
    }

    public static Translation2d mirrorTranslation(Translation2d translation) {
        return new Translation2d(translation.getX(), FlippingUtil.fieldSizeY - translation.getY());
    }

    public static Pose2d mirrorPose2d(Pose2d pose){
        return new Pose2d( mirrorTranslation(pose.getTranslation()), pose.getRotation().unaryMinus());
    }
}
