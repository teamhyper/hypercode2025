package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

/*
 * A command that rotates the robot to face a specific AprilTag.
 */
public class RotateToFaceAprilTagCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final Drivetrain drivetrain;
    private final int targetTag;
    private final double angleTolerance = 5.0; // Adjust as needed
    private boolean isAligned;

    /**
     * Constructs a new RotateToFaceAprilTagCommand that rotates the robot to face a
     * specific AprilTag.
     * @param visionSubsystem The VisionSubsystem to use.
     * @param drivetrain The Drivetrain to use.
     * @param targetTag The target AprilTag to face.
     */
    public RotateToFaceAprilTagCommand(VisionSubsystem visionSubsystem, Drivetrain drivetrain, int targetTag) {
        this.visionSubsystem = visionSubsystem;
        this.drivetrain = drivetrain;
        this.isAligned = false;
        this.targetTag = targetTag;
        addRequirements(visionSubsystem, drivetrain);
    }

    @Override
    public void execute() {
        List<PhotonTrackedTarget> targetList = visionSubsystem.getForwardTags();
        if (targetList.size() > 0) {
            final PhotonTrackedTarget target = targetList.get(0);
            double relativeYaw = target.getYaw();
            double currentYaw = drivetrain.getPose().getRotation().getDegrees();
            double targetAbsYaw = currentYaw - relativeYaw - 25.0;
            drivetrain.turnToAngle(Rotation2d.fromDegrees(targetAbsYaw));
        }
    }

    @Override
    public void end(boolean interrupted) {
        // no-op
    }

    @Override
    public boolean isFinished() {
        List<PhotonTrackedTarget> targetList = visionSubsystem.getForwardTags();
        if (targetList.size() > 0) {
            final PhotonTrackedTarget target = targetList.get(0);
            double relativeYaw = target.getYaw();
            double currentYaw = drivetrain.getPose().getRotation().getDegrees();
            double targetAbsYaw = currentYaw - relativeYaw - 25.0;

            isAligned = Math.abs(targetAbsYaw - currentYaw) < angleTolerance;
        }
        return isAligned;
    }

    private static void rotateToFaceAllianceReef(DriverStation.Alliance alliance){
        List<Integer> redAllianceTags = new ArrayList<>(Arrays.asList(6,7,8,9,10,11));
        List<Integer> blueAllianceTags = new ArrayList<>(Arrays.asList(17,18,19,20,21,22));

        if (alliance == DriverStation.Alliance.Red){

        } else {

        }
    }
}