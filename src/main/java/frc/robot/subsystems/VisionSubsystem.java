package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;
import java.util.ArrayList;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * A subsystem that handles vision processing.
 */
public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera rearCamera;
    private final PhotonCamera forwardCamera;
    private final Drivetrain drivetrain;
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new Transform3d()
    );

    /**
     * Constructs a new VisionSubsystem.
     *
     * @param drivetrain the drivetrain subsystem.
     */
    public VisionSubsystem(Drivetrain drivetrain) {
        forwardCamera = new PhotonCamera("hypercam-01");
        rearCamera = new PhotonCamera("hypercam-02");
        this.drivetrain = drivetrain;
        // Attempt to retrieve an immediate vision measurement from the forward camera.
        getPhotonPosition().ifPresent(visionMeasurement ->
            drivetrain.resetPose(visionMeasurement.estimatedPose.toPose2d())
        );
        if (!getPhotonPosition().isPresent()) {
            drivetrain.resetPose(new edu.wpi.first.math.geometry.Pose2d());
        }
    }

    @Override
    public void periodic() {
        // No periodic update; camera data is retrieved on-demand.
    }

    /**
     * Retrieves the current estimated robot pose based on the forward camera's latest results.
     *
     * @return An Optional containing the estimated robot pose, if available.
     */
    public Optional<EstimatedRobotPose> getPhotonPosition() {
        List<PhotonPipelineResult> results = getForwardCameraResults();
        if (results.isEmpty()) {
            return Optional.empty();
        }
        return poseEstimator.update(results.get(results.size() - 1));
    }

    /**
     * Retrieves the latest rear camera results.
     *
     * @return The latest rear camera results.
     */
    public List<PhotonPipelineResult> getRearCameraResults() {
        return rearCamera.getAllUnreadResults();
    }

    /**
     * Retrieves the latest forward camera results.
     *
     * @return The latest forward camera results.
     */
    public List<PhotonPipelineResult> getForwardCameraResults() {
        List<PhotonPipelineResult>results = forwardCamera.getAllUnreadResults();
        System.out.println(results.toString());
        return results;
    }

    /**
     * Returns a list of current forward camera targets.
     *
     * @return A list of PhotonTrackedTarget currently visible in the forward camera.
     */
    public List<PhotonTrackedTarget> getForwardTags() {
        List<PhotonPipelineResult> results = getForwardCameraResults();
        if (!results.isEmpty()) {
            PhotonPipelineResult latestFrame = results.get(results.size() - 1);
            if (latestFrame.hasTargets()) {
                return new ArrayList<>(latestFrame.targets);
            }
        }
        return new ArrayList<>();
    }

    /**
     * Returns a list of current rear camera targets.
     *
     * @return A list of PhotonTrackedTarget currently visible in the rear camera.
     */
    public List<PhotonTrackedTarget> getRearTags() {
        List<PhotonPipelineResult> results = rearCamera.getAllUnreadResults();
        if (!results.isEmpty()) {
            PhotonPipelineResult latestFrame = results.get(results.size() - 1);
            if (latestFrame.hasTargets()) {
                return new ArrayList<>(latestFrame.targets);
            }
        }
        return new ArrayList<>();
    }

    /**
     * Gets the rear camera used by the VisionSubsystem.
     *
     * @return The rear PhotonCamera.
     */
    public PhotonCamera getRearCamera() {
        return rearCamera;
    }

    /**
     * Gets the forward camera used by the VisionSubsystem.
     *
     * @return The forward PhotonCamera.
     */
    public PhotonCamera getForwardCamera() {
        return forwardCamera;
    }
}
