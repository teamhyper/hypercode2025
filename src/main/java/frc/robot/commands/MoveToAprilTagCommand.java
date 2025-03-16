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
 * distance matches a desired distance and rotates the robot such that the target’s yaw error is minimized.
 * The drivetrain is controlled via a SwerveRequest that is instantiated once and mutated during execution.
 */
public class MoveToAprilTagCommand extends Command {
    private final Drivetrain drivetrain;
    private final VisionSubsystem vision;
    private final double desiredDistance; // desired distance from the tag (meters)
    private final double maxSpeed;        // drivetrain's max speed in m/s

    // PID controllers for forward (distance) and rotation (angle) control.
    private final PIDController distanceController = new PIDController(10.0, 1.5, 1.5);
    private final PIDController angleController = new PIDController(1.0, 0.2, 0.0);

    // Tolerances to determine when the command should finish.
    private static final double DISTANCE_TOLERANCE_METERS = 0.05; // 5 cm tolerance
    private static final double ANGLE_TOLERANCE_RADIANS = Math.toRadians(2); // 2 degrees tolerance

    // SwerveRequest is instantiated once and then mutated using withX functions.
    private SwerveRequest.RobotCentricFacingAngle robotFacingRequest;

    /**
     * Constructs a new MoveToAprilTagCommand.
     *
     * @param drivetrain the drivetrain subsystem.
     * @param vision the vision subsystem.
     * @param desiredDistance the desired distance from the AprilTag (in meters).
     * @param maxSpeed the drivetrain's maximum speed (in m/s).
     */
    public MoveToAprilTagCommand(Drivetrain drivetrain, VisionSubsystem vision, double desiredDistance, double maxSpeed) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.desiredDistance = desiredDistance;
        this.maxSpeed = maxSpeed;
        addRequirements(drivetrain);

        distanceController.setTolerance(DISTANCE_TOLERANCE_METERS);
        angleController.setTolerance(ANGLE_TOLERANCE_RADIANS);
    }

    @Override
    public void initialize() {
        // Reset the PID controllers.
        distanceController.reset();
        angleController.reset();
        // Instantiate the SwerveRequest only once.
        robotFacingRequest = new SwerveRequest.RobotCentricFacingAngle();
    }

    @Override
    public void execute() {
        // Retrieve the list of detected targets.
        List<PhotonTrackedTarget> targets = vision.getForwardTags();

        if (targets.isEmpty()) {
            return;
        }

        // Use the first detected target.
        PhotonTrackedTarget target = targets.get(0);
        
        // Retrieve the relative transform from the camera to the target.
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        double forwardDistance = cameraToTarget.getTranslation().getX();

        // Convert the target yaw error from degrees to radians.
        double yawErrorRadians = Math.toRadians(target.getYaw());

        // Compute the forward PID output (negative to drive toward the desired distance).
        double pidOutput = -distanceController.calculate(forwardDistance, desiredDistance);

        // Clamp the PID output to the range [-1, 1] and convert it to a speed (m/s).
        double forwardSpeed = Math.max(-1.0, Math.min(1.0, pidOutput)) * maxSpeed;

        // Publish telemetry for debugging.
        SmartDashboard.putNumber("Forward PID Output", pidOutput);
        SmartDashboard.putNumber("Forward Speed (m/s)", forwardSpeed);
        SmartDashboard.putNumber("Forward Distance (m)", forwardDistance);
        SmartDashboard.putNumber("Yaw Error (rad)", yawErrorRadians);

        // Mutate the SwerveRequest using the withX functions.
        robotFacingRequest.withVelocityX(forwardSpeed)
                          .withTargetDirection(new Rotation2d(yawErrorRadians));

        // Apply the updated SwerveRequest to control the drivetrain.
        drivetrain.setControl(robotFacingRequest);
    }

    @Override
    public boolean isFinished() {
        // This command is held while the trigger is true.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends.
        drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(new ChassisSpeeds(0, 0, 0)));
    }
}
