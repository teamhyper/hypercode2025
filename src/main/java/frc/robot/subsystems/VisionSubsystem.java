package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/*
 * A subsystem that handles vision processing.
 */
public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera_2;
    private final PhotonCamera camera_1;
    private List<PhotonPipelineResult> latestResults;

    /**
     * Constructs a new VisionSubsystem.
     */
    public VisionSubsystem() {
        camera_2 = new PhotonCamera("arducam-ov9281-usb-02");
        camera_1 = new PhotonCamera("arducam-ov9281-usb-01");
    }

    @Override
    public void periodic() {
        updateLatestResults();
        // Check if tag 16 is in view and put the result on the SmartDashboard
        SmartDashboard.putBoolean("Tag 16 Visible", tagInView(16));
        // Report the yaw of tag 16 to SmartDashboard
        getTagIfInView(16).ifPresent(tag -> SmartDashboard.putNumber("Vision Tag 16 Yaw", tag.getYaw()));

    }

    /**
     * Updates the latest results from the camera.
     * This method should be called periodically.
     */
    public void updateLatestResults() {
        latestResults = camera_2.getAllUnreadResults();
    }

    /**
     * Gets the latest results from the camera.
     * @return The latest results from the camera.
     */
    public List<PhotonPipelineResult> getLatestResults() {
        return latestResults;
    }

    /**
     * Checks if a specific tag is in the current view.
     * @param fiducialId The ID of the tag to check for.
     * @return Whether the tag is in the current view.
     */
    public boolean tagInView(int fiducialId) {
        return getTagIfInView(fiducialId).isPresent();
    }

    public Optional<PhotonTrackedTarget> getTagIfInView(int fiducialId) {
        if (latestResults.isEmpty()) {
            return Optional.empty();
        }
        final var frame = latestResults.get(latestResults.size() - 1);
        if (frame.hasTargets()) {
            for (var target : frame.targets) {
                if (target.getFiducialId() == fiducialId) {
                    return Optional.of(target);
                }
            }
        }

        return Optional.empty();
    }

    /**
     * Gets the camera used by the VisionSubsystem.
     * @return The camera used by the VisionSubsystem.
     */
    public PhotonCamera getCamera_2() {
        return camera_2;
    }

    public PhotonCamera getCamera_1() {
        return camera_1;
    }
}