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

enum Side {left, right};

public class MoveToTagCommand extends Command{
    private final Vision vision;
    private final Drivetrain drivetrain;
    private final Side side;

    PhotonTrackedTarget target;
    int targetID;
    

    double targetDistance = .2;
    
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

    SwerveRequest.RobotCentric robotCentricDrive;

    private List<PhotonPipelineResult> results;
    private final SwerveRequest.RobotCentric driveRequest;

    public MoveToTagCommand(Vision vision, Drivetrain drivetrain, Side side) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.side = side;
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
        cameraToTagPose = new Pose3d(cameraToTargetTransform.getTranslation(), cameraToTargetTransform.getRotation());
        robotToCameraPose = new Pose3d(robotToCameraTransform.getTranslation(), robotToCameraTransform.getRotation());
        robotToTagPose = robotToCameraPose.transformBy(cameraToTargetTransform);
        robotToDesiredPose = robotToTagPose.transformBy(new Transform3d(targetPoseRelativeToTag.getTranslation(), targetPoseRelativeToTag.getRotation()));

        translationError = robotToDesiredPose.getTranslation();
        rotationError = robotToDesiredPose.getRotation();

        dx = translationError.getX();
        dy = translationError.getY();

        angleError = rotationError.getZ();
    }

    @Override
    public void execute() {
        results = vision.getLatestCamera1Results();
        if (results.get(results.size()-1).hasTargets()) {
            target = results.get(0).getBestTarget();
            targetID = target.getFiducialId();
            // cameraToTarget = target.getBestCameraToTarget();
        }

        double kLinear = 0.5;
    double kAngular = 2.0;
    double vx = kLinear * dx;
    double vy = kLinear * dy;
    double omega = kAngular * angleError;

        drivetrain.setControl(driveRequest
            .withVelocityX(null)
        );
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

    
}
