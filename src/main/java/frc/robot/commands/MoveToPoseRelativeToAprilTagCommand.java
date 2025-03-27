package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

/**
 * Command to move the robot to a specified pose relative to an AprilTag.
 */
public class MoveToPoseRelativeToAprilTagCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final Drivetrain drivetrain;
    private final int targetTag;
    private final Pose2d relativePose;
    private final double positionTolerance = 0.2; // Adjust as needed
    private final double angleTolerance = 5.0; // Adjust as needed
    private boolean isAligned;
    private Pose2d initialPose;
    private Pose2d finalPose;

    /**
     * Constructor for the MoveToPoseRelativeToAprilTagCommand.
     * 
     * @param visionSubsystem The VisionSubsystem instance.
     * @param drivetrain The Drivetrain instance.
     * @param targetTag The target AprilTag ID.
     * @param relativePose The desired pose relative to the AprilTag.
     */
    public MoveToPoseRelativeToAprilTagCommand(VisionSubsystem visionSubsystem, Drivetrain drivetrain, int targetTag, Pose2d relativePose) {
        this.visionSubsystem = visionSubsystem;
        this.drivetrain = drivetrain;
        this.targetTag = targetTag;
        this.relativePose = relativePose;
        this.isAligned = false;
        addRequirements(visionSubsystem, drivetrain);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("MoveToPoseRelativeToAprilTagCommand Running", true);
        this.initialPose = drivetrain.getPose();
        SmartDashboard.putNumber("Initial X", initialPose.getX());
        SmartDashboard.putNumber("Initial Y", initialPose.getY());
        Optional<PhotonTrackedTarget> targetOpt = visionSubsystem.getTagIfInView(targetTag);
        if (targetOpt.isPresent()) {
            PhotonTrackedTarget target = targetOpt.get();
            double targetX = target.getBestCameraToTarget().getX();
            double targetY = target.getBestCameraToTarget().getY();
            double targetYaw = target.getYaw();
    
            // Convert target data to a Pose2d in the robot's coordinate frame
            Pose2d cameraToTarget = new Pose2d(targetX, targetY, Rotation2d.fromDegrees(targetYaw));

            // Apply the desired relative pose
            finalPose = cameraToTarget.transformBy(new Transform2d(
                relativePose.getTranslation(),
                relativePose.getRotation()
            ));
            SmartDashboard.putNumber("Final X", finalPose.getX());
            SmartDashboard.putNumber("Final Y", finalPose.getY());

        } else {
            // If the target is not seen, cancel the command
            cancel();
        }
    } 

    @Override
    public void execute() {
        if (finalPose != null) {
            // Use the initial pose to drive the robot to the specified pose
            drivetrain.moveToPose(finalPose);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = drivetrain.getPose();
        boolean positionReached = Math.abs(currentPose.getX() - finalPose.getX()) < positionTolerance &&
                                  Math.abs(currentPose.getY() - finalPose.getY()) < positionTolerance;
        boolean angleAligned = Math.abs(currentPose.getRotation().getDegrees() - finalPose.getRotation().getDegrees()) < angleTolerance;

        isAligned = positionReached && angleAligned;
        return isAligned;
    }
}