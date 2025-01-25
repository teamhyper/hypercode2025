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
    private final PhotonCamera camera_1;
    private List<PhotonPipelineResult> latestResults;

    /**
     * Constructs a new VisionSubsystem.
     */
    public VisionSubsystem() {
        camera_1 = new PhotonCamera("photonvision");
        latestResults = camera_1.getAllUnreadResults();
    }

    @Override
    public void periodic() {
        updateLatestResults();
        // Report all tags in view to SmartDashboard
        SmartDashboard.putNumber("Vision Tags In View", getAllTagsInView().size());
        // Report every tag in view to SmartDashboard
        getAllTagsInView().forEach(tag -> SmartDashboard.putBoolean("Vision Tag " + tag + " In View", true));
        // Check if tag 16 is in view and put the result on the SmartDashboard
        SmartDashboard.putBoolean("Vision Has Target", tagInView(16));
        // Report the yaw of tag 16 to SmartDashboard
        getTagIfInView(16).ifPresent(tag -> SmartDashboard.putNumber("Vision Tag 16 Yaw", tag.getYaw()));

    }

    /**
     * Updates the latest results from the camera.
     */
    public void updateLatestResults() {
        latestResults = camera_1.getAllUnreadResults();
    }

    /**
     * Gets the latest results from the camera.
     */
    public List<PhotonPipelineResult> getLatestResults() {
        if (latestResults.isEmpty()) {
            return null;
        }
        return latestResults;
    }

    /**
     * Gets the tags in the current view.
     */
    public List<Integer> getAllTagsInView() {
        List<Integer> tags = List.of();

        if (latestResults.isEmpty()) {
            return tags;
        }


        final var frame = latestResults.get(latestResults.size() - 1);

        if (frame.hasTargets()) {
            for (var target : frame.getTargets()) {
                tags.add(target.getFiducialId());
            }
        }

        return tags;
    }

    /**
     * Checks if a specific tag is in the current view.
     * @param fiducialId The ID of the tag to check for.
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
            for (var target : frame.getTargets()) {
                if (target.getFiducialId() == fiducialId) {
                    return Optional.of(target);
                }
            }
        }

        return Optional.empty();
    }

    /**
     * Gets the camera used by the VisionSubsystem.
     */
    public PhotonCamera getCamera_1() {
        return camera_1;
    }
}