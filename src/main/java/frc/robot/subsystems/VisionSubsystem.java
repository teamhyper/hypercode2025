package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Collection;
import java.util.Collections;
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
public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera_2;
    private final PhotonCamera camera_1;
    private final Drivetrain drivetrain;
    private List<PhotonPipelineResult> latestCamera1Results = List.of();
    private List<PhotonPipelineResult> latestCamera2Results = List.of();
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d());

    /**
     * Constructs a new VisionSubsystem.
     */
    public VisionSubsystem(Drivetrain drivetrain) {
        camera_2 = new PhotonCamera("hypercam-02");
        camera_1 = new PhotonCamera("hypercam-01");
        var visionMeasurement = getPhotonPosition().orElse(null);
        if (visionMeasurement != null) {
            drivetrain.resetPose(visionMeasurement.estimatedPose.toPose2d());
        } else {
            drivetrain.resetPose(new edu.wpi.first.math.geometry.Pose2d());
        }
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        updateLatestCameraResults();
        var visionMeasurement = getPhotonPosition().orElse(null);
        if (visionMeasurement != null) {
            SmartDashboard.putNumber("Vision X", visionMeasurement.estimatedPose.getTranslation().getX());
            SmartDashboard.putNumber("Vision Y", visionMeasurement.estimatedPose.getTranslation().getY());
            SmartDashboard.putNumber("Vision Rotation", visionMeasurement.estimatedPose.getRotation().getAngle());

            var visionMeasurement2d = visionMeasurement.estimatedPose.toPose2d();

            drivetrain.addVisionMeasurement(visionMeasurement2d, 0);
        }

        if (!getAllTagsInCamera1View().isEmpty()) {
            for (PhotonTrackedTarget target : getAllTagsInCamera1View()) {
                System.out.println(target.getFiducialId());
            }
        }

        // Check if tag 16 is in view and put the result on the SmartDashboard
        SmartDashboard.putBoolean("Tag 16 Visible", tagInView(16));
        // Report the yaw of tag 16 to SmartDashboard
        getTagIfInView(16).ifPresent(tag -> SmartDashboard.putNumber("Vision Tag 16 Yaw", tag.getYaw()));

    }

    /**
     * Updates the latest results from the camera.
     * This method should be called periodically.
     */
    public void updateLatestCameraResults() {
        latestCamera1Results = camera_1.getAllUnreadResults();
        latestCamera2Results = camera_2.getAllUnreadResults();
    }

    public Optional<EstimatedRobotPose> getPhotonPosition() {
        if (latestCamera1Results == null || latestCamera1Results.isEmpty()) {
            return Optional.empty();
        }
        return poseEstimator.update(latestCamera1Results.get(latestCamera1Results.size() - 1));
    }

    /**
     * Gets the latest results from the camera.
     * @return The latest results from the camera.
     */
    public List<PhotonPipelineResult> getLatestCamera2Results() {
        return latestCamera2Results;
    }

    public List<PhotonPipelineResult> getLatestCamera1Results() {
        return latestCamera1Results;
    }

    /**
     * Checks if a specific tag is in the current view.
     * @param fiducialId The ID of the tag to check for.
     * @return Whether the tag is in the current view.
     */
    public boolean tagInView(int fiducialId) {
        return getTagIfInView(fiducialId).isPresent();
    }

    public List<PhotonTrackedTarget> getAllTagsInCamera1View(){
        if (!latestCamera1Results.isEmpty()) {
            final var frame = latestCamera1Results.get(latestCamera1Results.size() - 1);
            if (frame.hasTargets()) {
                return frame.getTargets();
            }
        }
        return Collections.emptyList();
    }

    public Optional<PhotonTrackedTarget> getTagIfInView(int fiducialId) {
        if (latestCamera2Results.isEmpty() && latestCamera1Results.isEmpty()) {
            return Optional.empty();
        }

        if (!latestCamera1Results.isEmpty()) {
            final var frame1 = latestCamera1Results.get(latestCamera1Results.size() - 1);
            if (frame1.hasTargets()) {
                for (var target : frame1.targets) {
                    if (target.getFiducialId() == fiducialId) {
                        return Optional.of(target);
                    }
                }
            }
        }
        
        if (!latestCamera2Results.isEmpty()) {
            final var frame2 = latestCamera2Results.get(latestCamera2Results.size() - 1);
            if (frame2.hasTargets()) {
                for (var target : frame2.targets) {
                    if (target.getFiducialId() == fiducialId) {
                        return Optional.of(target);
                    }
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