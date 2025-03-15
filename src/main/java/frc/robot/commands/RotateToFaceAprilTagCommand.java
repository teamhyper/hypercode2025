package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;

/*
 * A command that rotates the robot to face a specific AprilTag.
 */
public class RotateToFaceAprilTagCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final Drivetrain drivetrain;
    private final PIDController pidController; 
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
        pidController = new PIDController(0.01, 0.08, 0.05);
        pidController.setTolerance(4.0);
        addRequirements(visionSubsystem, drivetrain);
    }

    @Override
    public void execute() {
        List<PhotonTrackedTarget> targetList = visionSubsystem.getForwardTags();
        if (targetList.size() > 0) {
            final PhotonTrackedTarget target = targetList.get(targetList.size() - 1);
            double targetYaw = target.getYaw(); 
            // double currentYaw = drivetrain.getPose().getRotation().getDegrees();
            // double targetAbsYaw = currentYaw - relativeYaw;
            double pidOutput = pidController.calculate(targetYaw, 0);

            SmartDashboard.putNumber("pidOutput", pidOutput);
            SmartDashboard.putNumber("targetYaw", targetYaw);
            // SmartDashboard.putNumber("currentYaw", currentYaw);
            // SmartDashboard.putNumber("targetAbsYaw", targetAbsYaw);

            // Rotation2d rotationCommand = Rotation2d.fromDegrees(pidOutput);

            final var chassisSpeeds = new ChassisSpeeds(0, 0, pidOutput);
            drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(chassisSpeeds));
            // drivetrain.applyRequest(null)
        } else {
            final var chassisSpeeds = new ChassisSpeeds(0, 0, 0);
            drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(chassisSpeeds));
        }
    }

    @Override
    public void end(boolean interrupted) {
        // no-op
    }

    // @Override
    // public boolean isFinished() {
    //     List<PhotonTrackedTarget> targetList = visionSubsystem.getForwardTags();
    //     if (targetList.size() > 0) {
    //         final PhotonTrackedTarget target = targetList.get(targetList.size() - 1);
    //         double relativeYaw = target.getYaw();
    //         double currentYaw = drivetrain.getPose().getRotation().getDegrees();
    //         double targetAbsYaw = currentYaw - relativeYaw;

    //         isAligned = Math.abs(targetAbsYaw - currentYaw) < angleTolerance;
    //     }
    //     return isAligned;
    // }
}