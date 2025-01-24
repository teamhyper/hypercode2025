package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class DriveToAprilTagCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final CommandSwerveDrivetrain drivetrain;
    private Pose2d targetPose;

    public DriveToAprilTagCommand(VisionSubsystem visionSubsystem, CommandSwerveDrivetrain drivetrain) {
        this.visionSubsystem = visionSubsystem;
        this.drivetrain = drivetrain;
        addRequirements(visionSubsystem, drivetrain);
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = visionSubsystem.getLatestResult();
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            // Process the target data to drive to the desired location
            var targetX = target.getBestCameraToTarget().getX();
            var targetY = target.getBestCameraToTarget().getY();
            var targetYaw = target.getYaw();

            // Convert target data to a Pose2d
// Convert target data to a Pose2d in the robot's coordinate frame
            var cameraToTarget = new Pose2d(targetX, targetY, Rotation2d.fromDegrees(targetYaw));
            var robotPose = drivetrain.getPose();
            var fieldToTarget = robotPose.transformBy(new Transform2d(cameraToTarget.getTranslation(), cameraToTarget.getRotation()));
            targetPose = fieldToTarget;
            // Use the target data to drive the robot
            drivetrain.driveToPosition(fieldToTarget);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            // Align the robot to face the AprilTag
            var result = visionSubsystem.getLatestResult();
            if (result.hasTargets()) {
                var target = result.getBestTarget();
                double targetYaw = target.getYaw();
                drivetrain.turnToAngle(Rotation2d.fromDegrees(targetYaw));
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Check if the robot has reached the target position and is aligned
        var currentPose = drivetrain.getPose();
        var positionTolerance = 0.1; // Adjust as needed
        var angleTolerance = 5.0; // Adjust as needed

        var positionReached = Math.abs(currentPose.getX() - targetPose.getX()) < positionTolerance &&
                                  Math.abs(currentPose.getY() - targetPose.getY()) < positionTolerance;
        var angleAligned = Math.abs(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees()) < angleTolerance;

        return positionReached && angleAligned;
    }
}