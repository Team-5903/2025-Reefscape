// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Util.AprilTagCamera;
import frc.robot.Robot;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionOdometryHelper extends Command {
  /** Creates a new VisionOdometryHelper. */
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
              new Rotation3d(0, Math.toRadians(-30), Math.toRadians(-90))
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
              new Rotation3d(0, Math.toRadians(-30), Math.toRadians(-90))
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
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
