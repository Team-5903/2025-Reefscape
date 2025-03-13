// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.util.FlippingUtil;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Util.AprilTagCamera;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionOdometryHelper extends Command {
  /** Creates a new VisionOdometryHelper. */
  private static final double xyStdDevCoefficient = 0.5;
  private List<AprilTagCamera> cameras;
  private VisionSystemSim     visionSim;
  private final SwerveSubsystem swerve;

  public VisionOdometryHelper(SwerveSubsystem swerve) {
    this.swerve = swerve;

    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    this.cameras = new ArrayList<>(){
      {
        add(new AprilTagCamera("LeftCamera", 
          new PhotonPoseEstimator(fieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            new Transform3d(
              new Translation3d(Units.inchesToMeters(3.812),
                                Units.inchesToMeters(8.729),
                                Units.inchesToMeters(8.747)),
              new Rotation3d(0, Math.toRadians(-30), Math.toRadians(90))
            )
          ), PoseStrategy.CLOSEST_TO_REFERENCE_POSE)
        );
        add(new AprilTagCamera("RightCamera", 
          new PhotonPoseEstimator(fieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            new Transform3d(
              new Translation3d(Units.inchesToMeters(3.812),
                                Units.inchesToMeters(-8.729),
                                Units.inchesToMeters(8.747)),
              new Rotation3d(0, Math.toRadians(-30), Math.toRadians(-85))
            )
          ), PoseStrategy.CLOSEST_TO_REFERENCE_POSE)
        );
        add(new AprilTagCamera("FrontCamera", 
          new PhotonPoseEstimator(fieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            new Transform3d(
              new Translation3d(Units.inchesToMeters(11.75),
                                Units.inchesToMeters(1.875),
                                Units.inchesToMeters(15.75)),
              new Rotation3d(0, Units.degreesToRadians(0), 0)
            )
          ), PoseStrategy.CLOSEST_TO_REFERENCE_POSE)
        );
      }
    };

    if (Robot.isSimulation())
    {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      this.cameras.forEach(x -> visionSim.addCamera(x.cameraSim, x.getTransform()));


      // openSimCameraViews();
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.isSimulation())
    {
      visionSim.update(swerve.getPose());
    }


    cameras.forEach(x -> {

      List<Double> averageDistanceToTargetList = new ArrayList<>();
      List<Double> averageAmbiguityList = new ArrayList<>();
      List<PhotonTrackedTarget> trackedTargets = new ArrayList<>();
      List<Pose2d> visionOdometryList = new ArrayList<>();
      List<Double> timeStampList = new ArrayList<>();
      List<Matrix<N3, N1>> matrixList = new ArrayList<>();

      x.getRobotPoses(
        this.swerve.getPose(), 
        pose -> {
          if(pose.targetsUsed.size() == 0)//need at least 1 tag
          {
            return false;
          }

          if(pose.estimatedPose.getX() < 0 || pose.estimatedPose.getY() < 0)//estimated pose is outside field
          {
            return false;
          }

          if(pose.estimatedPose.getX() > FlippingUtil.fieldSizeX || pose.estimatedPose.getY() > FlippingUtil.fieldSizeY)//estimated pose is outside field
          {
            return false;
          }

          if(Math.abs(pose.estimatedPose.getZ()) > Constants.VisionConstants.MAX_Z_ERROR)//estimated pose is not close to ground
          {
            return false;
          }

          return true;
        }
      ).forEach(pose -> {

          double averageDistanceToTarget = pose.targetsUsed
            .stream()
            .map(z -> z.getBestCameraToTarget().getTranslation())
            .mapToDouble(Translation3d::getNorm)
            .average()
            .getAsDouble();
          double averageAmbiguity = pose.targetsUsed
            .stream()
            .mapToDouble(z -> z.getPoseAmbiguity())
            .average()
            .getAsDouble();

          averageDistanceToTargetList.add(averageDistanceToTarget);
          averageAmbiguityList.add(averageAmbiguity);
          timeStampList.add(pose.timestampSeconds);
          // Logger.recordOutput("Vision/averageDistanceToTarget", averageDistanceToTarget);
          // Logger.recordOutput("Vision/averageAmbiguity", averageAmbiguity);
          // double poseSTD =
          // interp.get(averageDistanceToTarget)/Math.pow(pose.targetsUsed.size(), 3);
          double poseSTD = xyStdDevCoefficient 
            * Math.pow(averageDistanceToTarget, 3)
            * (1 / Math.pow(pose.targetsUsed.size() * 3, 3))
            * (averageAmbiguity * 10);
        
          Matrix<N3, N1> matrix = VecBuilder.fill(poseSTD, poseSTD, Double.MAX_VALUE);
          matrixList.add(matrix);

          swerve.getSwerveDrive().addVisionMeasurement(
            pose.estimatedPose.toPose2d(), 
            pose.timestampSeconds,
            matrix
          );
          visionOdometryList.add(pose.estimatedPose.toPose2d());
          trackedTargets.addAll(pose.targetsUsed);
      });

      Logger.recordOutput("Vision/" + x.name + "/Targets", trackedTargets
        .stream()
        .distinct()
        .map(z -> cameras.get(0).poseEstimator.getFieldTags().getTagPose(z.getFiducialId()).get())
        .toList()
        .toArray(Pose3d[]::new)
      );
      Logger.recordOutput("Vision/" + x.name + "/CameraLocation", new Pose3d(swerve.getPose()).plus(x.getTransform()));
      Logger.recordOutput("Vision/" + x.name + "/isConnected", x.camera.isConnected());
      Logger.recordOutput("Vision/" + x.name + "/Poses", visionOdometryList.toArray(Pose2d[]::new));
      Logger.recordOutput("Vision/" + x.name + "/Timestamps", timeStampList.stream().mapToDouble(z -> z).toArray());
      Logger.recordOutput("Vision/" + x.name + "/averageDistanceToTargets", averageDistanceToTargetList.stream().mapToDouble(z -> z).toArray());
      Logger.recordOutput("Vision/" + x.name + "/averageAmbiguitys", averageAmbiguityList.stream().mapToDouble(z -> z).toArray());
      Logger.recordOutput("Vision/" + x.name + "/StdDevs", matrixList.stream().map(z -> z.getData()).flatMapToDouble(Arrays::stream).toArray());


    });

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
