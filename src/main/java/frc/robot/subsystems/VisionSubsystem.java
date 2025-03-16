package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * A subsystem that handles vision processing with debouncing for tag detection.
 */
public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera rearCamera;
    private final PhotonCamera forwardCamera;
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new Transform3d()
    );

    // Debounce fields for forward targets.
    private double lastValidTagTime = 0.0;
    private List<PhotonTrackedTarget> lastValidTargets = new ArrayList<>();
    private static final double DEBOUNCE_TIME_SECONDS = 0.01; // 10 ms debounce window

    /**
     * Constructs a new VisionSubsystem.
     *
     * @param drivetrain the drivetrain subsystem.
     */
    public VisionSubsystem(Drivetrain drivetrain) {
        forwardCamera = new PhotonCamera("hypercam-01");
        rearCamera = new PhotonCamera("hypercam-02");
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
        List<PhotonPipelineResult> results = forwardCamera.getAllUnreadResults();
        if (results.isEmpty()) {
            return Optional.empty();
        }
        return poseEstimator.update(results.get(results.size() - 1));
    }

    /**
     * Returns a list of current forward camera targets with debouncing logic.
     * 
     * <p>If no new targets are detected but the last valid targets were seen within the debounce
     * window, the cached targets are returned.
     *
     * @return A list of PhotonTrackedTarget currently visible in the forward camera.
     */
    public List<PhotonTrackedTarget> getForwardTags() {
        List<PhotonPipelineResult> results = forwardCamera.getAllUnreadResults();
        double currentTime = Timer.getFPGATimestamp();
        if (!results.isEmpty()) {
            PhotonPipelineResult latestFrame = results.get(results.size() - 1);
            if (latestFrame.hasTargets()) {
                // Update the cache if targets are detected.
                lastValidTagTime = currentTime;
                lastValidTargets = new ArrayList<>(latestFrame.targets);
                return lastValidTargets;
            }
        }
        // Return cached targets if within debounce period.
        if (currentTime - lastValidTagTime < DEBOUNCE_TIME_SECONDS && !lastValidTargets.isEmpty()) {
            return lastValidTargets;
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
