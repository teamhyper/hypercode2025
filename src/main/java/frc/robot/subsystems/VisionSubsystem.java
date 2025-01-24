package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private List<PhotonPipelineResult> latestResults;

    public VisionSubsystem() {
        camera = new PhotonCamera("photonvision");
    }

    public void updateLatestResults() {
        latestResults = camera.getAllUnreadResults();
    }

    public List<PhotonPipelineResult> getLatestResults() {
        if (latestResults.isEmpty()) {
            return null;
        }
        return latestResults;
    }

    public PhotonCamera getCamera() {
        return camera;
    }
}