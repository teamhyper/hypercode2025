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
    private final double positionTolerance = 0.1; // Adjust as needed
    private final double angleTolerance = 5.0; // Adjust as needed
    private boolean isAligned;
    private Pose2d initialPose;

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
        SmartDashboard.putString("Relative Pose To Tag " + targetTag, relativePose.toString());
        Optional<PhotonTrackedTarget> targetOpt = visionSubsystem.getTagIfInView(targetTag);
        if (targetOpt.isPresent()) {
            PhotonTrackedTarget target = targetOpt.get();
            double targetX = target.getBestCameraToTarget().getX();
            double targetY = target.getBestCameraToTarget().getY();
            double targetYaw = target.getYaw();

            // Convert target data to a Pose2d in the robot's coordinate frame
            Pose2d cameraToTarget = new Pose2d(targetX, targetY, Rotation2d.fromDegrees(targetYaw));
            
            Transform2d cameraOffset = new Transform2d(
                new Translation2d(0, 0), // Replace with actual offsets
                Rotation2d.fromDegrees(0)          // Replace with actual yaw offset
            );
            
            Pose2d robotToTarget = cameraToTarget.transformBy(cameraOffset);

            Pose2d robotPose = drivetrain.getPose();
            // Pose2d fieldToTarget = robotPose.transformBy(new Transform2d(cameraToTarget.getTranslation(), cameraToTarget.getRotation()));
            Pose2d fieldToTarget = robotPose.transformBy(new Transform2d(robotToTarget.getTranslation(), robotToTarget.getRotation()));

            // Calculate the desired pose relative to the AprilTag
            initialPose = fieldToTarget.transformBy(new Transform2d(relativePose.getTranslation(), relativePose.getRotation()));
        } else {
            // If the target is not seen, cancel the command
            cancel();
        }
    } 

    @Override
    public void execute() {
        if (initialPose != null) {
            // Use the initial pose to drive the robot to the specified pose
            drivetrain.driveToPose(initialPose);
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("MoveToPoseRelativeToAprilTagCommand Running", false);
        if (!interrupted && initialPose != null) {
            // Align the robot to face the AprilTag
            double targetYaw = initialPose.getRotation().getDegrees();
            drivetrain.turnToAngle(Rotation2d.fromDegrees(targetYaw));
        }
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = drivetrain.getPose();
        boolean positionReached = Math.abs(currentPose.getX() - initialPose.getX()) < positionTolerance &&
                                  Math.abs(currentPose.getY() - initialPose.getY()) < positionTolerance;
        boolean angleAligned = Math.abs(currentPose.getRotation().getDegrees() - initialPose.getRotation().getDegrees()) < angleTolerance;

        isAligned = positionReached && angleAligned;
        return isAligned;
    }
}