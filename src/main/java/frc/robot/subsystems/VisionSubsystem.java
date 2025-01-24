package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;

    public VisionSubsystem() {
        camera = new PhotonCamera("photonvision");
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }
}