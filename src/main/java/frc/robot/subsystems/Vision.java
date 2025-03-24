package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/*
 * A subsystem that handles vision processing.
 */
public class Vision extends SubsystemBase {
    private final PhotonCamera camera_1;
    private final PhotonCamera camera_2;
    private final Drivetrain drivetrain;
    private List<PhotonPipelineResult> latestCamera1Results;
    private List<PhotonPipelineResult> latestCamera2Results;
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d());

    /**
     * Constructs a new Vision.
     */
    public Vision() {
        camera_1 = new PhotonCamera("hypercam-01");
        camera_2 = new PhotonCamera("hypercam-02");
        
        this.drivetrain = Drivetrain.getInstance();
        
        // var visionMeasurement = getPhotonPosition().orElse(null);
        // if (visionMeasurement != null) {
        //     drivetrain.resetPose(visionMeasurement.estimatedPose.toPose2d());
        // } else {
        //     drivetrain.resetPose(new edu.wpi.first.math.geometry.Pose2d());
        // }
        
    }

    @Override
    public void periodic() {
        updateLatestCameraResults();
    }

    /**
     * Updates the latest results from the camera.
     * This method should be called periodically.
     */
    public void updateLatestCameraResults() {
        latestCamera1Results = camera_1.getAllUnreadResults();
        // latestCamera2Results = camera_2.getAllUnreadResults();
    }

    /**
     * Gets the latest results from the camera.
     * @return The latest results from the camera.
     */
    public List<PhotonPipelineResult> getLatestCamera1Results() {
        return latestCamera1Results;
    }

    public List<PhotonPipelineResult> getLatestCamera2Results() {
        return latestCamera2Results;
    }    

    /**
     * Gets the camera used by the Vision.
     * @return The camera used by the Vision.
     */

     public PhotonCamera getCamera_1() {
        return camera_1;
    }

    public PhotonCamera getCamera_2() {
        return camera_2;
    }

    
}