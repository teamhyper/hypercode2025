package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;

/*
 * A command that rotates the robot to face a specific AprilTag.
 */
public class RotateToFaceAprilTagCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final CommandSwerveDrivetrain drivetrain;
    private final int targetTag;
    private final double angleTolerance = 5.0; // Adjust as needed
    private boolean isAligned;

    /**
     * Constructs a new RotateToFaceAprilTagCommand that rotates the robot to face a
     * specific AprilTag.
     * @param visionSubsystem The VisionSubsystem to use.
     * @param drivetrain The CommandSwerveDrivetrain to use.
     * @param targetTag The target AprilTag to face.
     */
    public RotateToFaceAprilTagCommand(VisionSubsystem visionSubsystem, CommandSwerveDrivetrain drivetrain, int targetTag) {
        this.visionSubsystem = visionSubsystem;
        this.drivetrain = drivetrain;
        this.isAligned = false;
        this.targetTag = targetTag;
        addRequirements(visionSubsystem, drivetrain);
    }

    @Override
    public void execute() {
        if (visionSubsystem.getTagIfInView(targetTag).isPresent()) {
            final var target = visionSubsystem.getTagIfInView(targetTag).get();
            final double targetYaw = target.getYaw();
            drivetrain.turnToAngle(Rotation2d.fromDegrees(targetYaw));
        }
    }

    @Override
    public void end(boolean interrupted) {
        // no-op
    }

    @Override
    public boolean isFinished() {
        if (visionSubsystem.getTagIfInView(targetTag).isPresent()) {
            final var target = visionSubsystem.getTagIfInView(targetTag).get();
            final double targetYaw = target.getYaw();
            final double currentYaw = drivetrain.getPose().getRotation().getDegrees();
            isAligned = Math.abs(currentYaw - targetYaw) < angleTolerance;
        }
        return isAligned;
    }
}