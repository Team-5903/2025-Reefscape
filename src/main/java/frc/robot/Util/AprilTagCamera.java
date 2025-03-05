package frc.robot.Util;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import frc.robot.Robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class AprilTagCamera {
    
    public final String name;

    public final  PhotonCamera                 camera;
    /**
     * Pose estimator for camera.
     */
    public final  PhotonPoseEstimator          poseEstimator;
    public        PhotonCameraSim              cameraSim;


    public AprilTagCamera(String name, PhotonPoseEstimator poseEstimator, PoseStrategy multiTagFallbackStrategy)
    {
        this.name = name;
        this.camera = new PhotonCamera(name);
        this.poseEstimator = poseEstimator;
        this.poseEstimator.setMultiTagFallbackStrategy(multiTagFallbackStrategy);

        
      if (Robot.isSimulation())
      {
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        this.cameraSim = new PhotonCameraSim(this.camera, cameraProp);
        this.cameraSim.enableDrawWireframe(true);
      }
    }

    public List<Optional<EstimatedRobotPose>> getRobotPoses()
    {
        return (Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults()).stream().map(x -> this.poseEstimator.update(x)).toList();
    }
}
