package frc.robot.Util;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

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
}
