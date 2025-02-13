package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Drivetrain;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

/**
 * A command that moves the robot to a specified pose if a specific AprilTag is seen.
 */
public class MoveToPoseIfAprilTagSeenCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final Drivetrain drivetrain;
    private final int targetTag;
    private final Pose2d targetPose;
    private final double positionTolerance = 0.1; // Adjust as needed
    private final double angleTolerance = 5.0; // Adjust as needed
    private boolean isAligned;
    private Pose2d initialPose;

    /**
     * Constructs a new MoveToPoseIfAprilTagSeenCommand.   
     * @param visionSubsystem The vision subsystem to use.
     * @param drivetrain The drivetrain to use.
     * @param targetTag The tag to look for.
     * @param targetPose The pose to move to if the tag is seen.
     */
    public MoveToPoseIfAprilTagSeenCommand(VisionSubsystem visionSubsystem, Drivetrain drivetrain, int targetTag, Pose2d targetPose) {
        this.visionSubsystem = visionSubsystem;
        this.drivetrain = drivetrain;
        this.targetTag = targetTag;
        this.targetPose = targetPose;
        this.isAligned = false;
        addRequirements(visionSubsystem, drivetrain);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("MoveToPoseIfAprilTagSeenCommand Running", true);
        SmartDashboard.putString("Target Pose", targetPose.toString());

        Optional<PhotonTrackedTarget> targetOpt = visionSubsystem.getTagIfInView(targetTag);
        if (targetOpt.isPresent()) {
            PhotonTrackedTarget target = targetOpt.get();
            double targetX = target.getBestCameraToTarget().getX();
            double targetY = target.getBestCameraToTarget().getY();
            double targetYaw = target.getYaw();

            // Convert target data to a Pose2d in the robot's coordinate frame
            initialPose = new Pose2d(targetX, targetY, Rotation2d.fromDegrees(targetYaw));
            SmartDashboard.putString("Initial Pose", initialPose.toString());
        } else {
            // If the target is not seen, cancel the command
            cancel();
        }
    }

    @Override
    public void execute() {
        if (initialPose != null) {
            // Use the initial pose to drive the robot to the specified pose
            drivetrain.driveToPose(targetPose);
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("MoveToPoseIfAprilTagSeenCommand Running", false);
        if (!interrupted && initialPose != null) {
            // Align the robot to face the AprilTag
            double targetYaw = initialPose.getRotation().getDegrees();
            drivetrain.turnToAngle(Rotation2d.fromDegrees(targetYaw));
        }
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = drivetrain.getPose();
        boolean positionReached = Math.abs(currentPose.getX() - targetPose.getX()) < positionTolerance &&
                                  Math.abs(currentPose.getY() - targetPose.getY()) < positionTolerance;
        boolean angleAligned = Math.abs(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees()) < angleTolerance;

        isAligned = positionReached && angleAligned;
        return isAligned;
    }
}