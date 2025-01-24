package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;

public class RotateToFaceAprilTagCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final CommandSwerveDrivetrain drivetrain;
    private boolean isAligned;

    public RotateToFaceAprilTagCommand(VisionSubsystem visionSubsystem, CommandSwerveDrivetrain drivetrain) {
        this.visionSubsystem = visionSubsystem;
        this.drivetrain = drivetrain;
        this.isAligned = false;
        addRequirements(visionSubsystem, drivetrain);
    }

    @Override
    public void execute() {
        var results = visionSubsystem.getLatestResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 16) {
                        double targetYaw = target.getYaw();
                        drivetrain.turnToAngle(Rotation2d.fromDegrees(targetYaw));
                    }
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // no-op
    }

    @Override
    public boolean isFinished() {
        var results = visionSubsystem.getLatestResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            
            var result = results.get(results.size() - 1);

            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 16) {
                        double targetYaw = target.getYaw();
                        double currentYaw = drivetrain.getPose().getRotation().getDegrees();
                        double angleTolerance = 5.0; // Adjust as needed
                        isAligned = Math.abs(currentYaw - targetYaw) < angleTolerance;
                    }
                }
                return isAligned;
            }
        }
        return false;
    }
}