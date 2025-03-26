package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.swerve.SwerveRequest;

import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivetrain;

// enum Side {left, right};

public class MoveToTagCommand extends Command{
    private final Vision vision;
    private final Drivetrain drivetrain;
    // private final Side side;

    private static final double TOLERENCE = .1;

    PhotonTrackedTarget target;
    int targetID;
    

    double targetDistance = .1;
    
    Transform3d cameraToTargetTransform;
    Transform3d robotToCameraTransform;
    Transform3d cameraToRobotTransform;
    Transform3d robotToTagTransform;

    Translation3d translationError;
    Rotation3d rotationError;

    Pose3d targetPoseRelativeToTag;
    Pose3d cameraToTagPose;
    Pose3d robotToCameraPose;
    Pose3d robotToTagPose;
    Pose3d robotToDesiredPose;

    double dx;
    double dy;
    double angleError;

    double kLinear;
    double kAngular;
    double vx;
    double vy;
    double omega;
    PhotonPipelineResult latestResult;


    SwerveRequest.RobotCentric robotCentricDrive;

    private List<PhotonPipelineResult> results;
    private final SwerveRequest.RobotCentric driveRequest;

    public MoveToTagCommand(Vision vision, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        // this.side = side;
        driveRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        targetPoseRelativeToTag = new Pose3d(
            new Translation3d(targetDistance ,0.0, 0.0),
            new Rotation3d(0.0, 0.0, Math.PI)
        );

        robotToCameraTransform = new Transform3d(
            new Translation3d(0.2, Units.inchesToMeters(11.0), 0.0),  // camera offset: +20cm forward, +50cm up
            new Rotation3d(0.0, 0.0, 0.0)      // camera facing same direction as robot
        );
        
        cameraToRobotTransform = robotToCameraTransform.inverse();
        
    }

    @Override
    public void execute() {
        results = vision.getLatestCamera1Results();        
        if (!results.isEmpty()) {
            latestResult = results.get(results.size() - 1);
            if (latestResult.hasTargets())
            target = latestResult.getBestTarget();
            targetID = target.getFiducialId();
            cameraToTargetTransform = target.getBestCameraToTarget();
            cameraToTagPose = new Pose3d(
                robotToCameraTransform.getTranslation(),
                robotToCameraTransform.getRotation()
            );
            robotToCameraPose = new Pose3d(robotToCameraTransform.getTranslation(), robotToCameraTransform.getRotation());
            robotToTagPose = robotToCameraPose.transformBy(new Transform3d(cameraToTagPose.getTranslation(), cameraToTagPose.getRotation()));
            robotToDesiredPose = robotToTagPose.transformBy(new Transform3d(targetPoseRelativeToTag.getTranslation(), targetPoseRelativeToTag.getRotation()));

            translationError = robotToDesiredPose.getTranslation();
            rotationError = robotToDesiredPose.getRotation();

            dx = translationError.getX();
            dy = translationError.getY();
            angleError = rotationError.getZ();
            kLinear = 5.0;
            kAngular = 0.1;
            vx = kLinear * dx;
            vy = kLinear * dy;
            omega = kAngular * angleError;

            drivetrain.setControl(driveRequest
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(omega)
            );
        }        
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return (Math.abs(vy - TOLERENCE) < .1) && 
            (Math.abs(vx - TOLERENCE) < .1) && 
            (Math.abs(omega - TOLERENCE) < .1);
    }

    
}
