package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * Command to drive the robot toward an AprilTag using relative measurements from PhotonVision.
 * 
 * <p>This command retrieves the relative transform from the forward camera to the target
 * AprilTag. It then uses a PID controller to drive forward until the camera-to-target forward 
 * distance matches a desired distance and a second PID controller to rotate the robot so that 
 * the target’s yaw error is minimized. Adjust the PID constants and tolerances as necessary.
 */
public class MoveToAprilTagCommand extends Command {
    private final Drivetrain drivetrain;
    private final VisionSubsystem vision;
    private final double desiredDistance; // desired distance from the tag (meters)

    // PID controllers for forward (distance) and rotation (angle) control.
    private final PIDController distanceController = new PIDController(10.0, 1.5, 1.5);
    private final PIDController angleController = new PIDController(1.0, 0.2, 0.0);

    // Tolerances to determine when the command should finish.
    private static final double DISTANCE_TOLERANCE_METERS = 0.2; // 10 cm
    private static final double ANGLE_TOLERANCE_RADIANS = Math.toRadians(5); // 5 degrees

    /**
     * Constructs a new MoveToAprilTagCommand.
     *
     * @param drivetrain the drivetrain subsystem.
     * @param vision the vision subsystem.
     * @param desiredDistance the desired distance from the AprilTag (in meters).
     */
    public MoveToAprilTagCommand(Drivetrain drivetrain, VisionSubsystem vision, double desiredDistance) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.desiredDistance = desiredDistance;
        addRequirements(drivetrain);

        distanceController.setTolerance(DISTANCE_TOLERANCE_METERS);
        angleController.setTolerance(ANGLE_TOLERANCE_RADIANS);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("forwardOutput", 0.0);
        SmartDashboard.putNumber("rotationOutput", 0.0);
        distanceController.reset();
        angleController.reset();
    }

    @Override
    public void execute() {
        List<PhotonTrackedTarget> targets = vision.getForwardTags();
        if (targets.isEmpty()) {
            // No target detected; stop the robot.
            drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds()
                    .withSpeeds(new ChassisSpeeds(0, 0, 0)));
            return;
        }

        // For simplicity, we select the first detected target.
        PhotonTrackedTarget target = targets.get(0);

        // Retrieve the relative transform from the camera to the target.
        // Assumes that the camera's coordinate system has X as the forward direction.
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        double forwardDistance = cameraToTarget.getTranslation().getX();

        // Get the relative yaw error (convert from degrees to radians).
        double yawErrorRadians = Math.toRadians(target.getYaw());

        // Compute the PID outputs.
        // For distance, we want the measured forward distance to converge to the desired distance.
        double forwardOutput = -distanceController.calculate(forwardDistance, desiredDistance);
        // For angle, we want the yaw error to reach 0 (i.e. the tag is centered).
        double rotationOutput = angleController.calculate(yawErrorRadians, 0);

        SmartDashboard.putNumber("forwardOutput", forwardOutput);
        SmartDashboard.putNumber("rotationOutput", rotationOutput);

        // Command the drivetrain with the computed forward and rotational speeds.
        // Lateral speed is set to 0 since we assume the control is robot-centric.
        drivetrain.setControl(
            new SwerveRequest.RobotCentricFacingAngle()
                .withVelocityX(forwardOutput)
                .withTargetDirection(new Rotation2d(rotationOutput))
            );
    }

    @Override
    public boolean isFinished() {
        List<PhotonTrackedTarget> targets = vision.getForwardTags();
        if (targets.isEmpty()) {
            return false;
        }

        // Finish when both the distance and angle controllers are at their setpoints.
        return distanceController.atSetpoint() && angleController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends.
        drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(new ChassisSpeeds(0, 0, 0)));
    }
}
