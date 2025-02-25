// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.filter.SlewRateLimiter;

import frc.robot.generated.TunerConstants;
import frc.robot.joysticks.ApemHF45Joystick;
import frc.robot.joysticks.VKBGladiatorJoystick;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Pivot;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Slew Rate Limiter
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(Constants.xSpeedLimiter);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(Constants.ySpeedLimiter);
    private final SlewRateLimiter rotLimiter    = new SlewRateLimiter(Constants.rotLimiter);

    /* Setting up bindings for necessary control of the swerve drive platform */
    // Field-Centric request
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 1% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // Robot-Centric request
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.01)
            .withRotationalDeadband(MaxAngularRate * 0.01)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controller Initialization
    ApemHF45Joystick driverJoystickLeft = new ApemHF45Joystick(0);
    ApemHF45Joystick driverJoystickRight = new ApemHF45Joystick(1);
    VKBGladiatorJoystick operatorJoystickLeft = new VKBGladiatorJoystick(2);
    VKBGladiatorJoystick operatorJoystickRight = new VKBGladiatorJoystick(3);

    // Subsytem Initialization
    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    public final EndEffector endEffector = new EndEffector();
    // public final VisionSubsystem vision = new VisionSubsystem();
    public final Elevator elevator = new Elevator();
    public final Pivot pivot = new Pivot();
    public final LEDStrip ledStrip = new LEDStrip();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // In your RobotContainer class, somewhere in configureBindings()

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                // Check which mode to use
                boolean isRobotCentric = false;
                // Check if slow mode is active
                boolean slowMode = false;

                boolean trackTag = driverJoystickRight.rightButton().getAsBoolean() || driverJoystickRight.leftButton().getAsBoolean();

                // Decide on your normal top speed/rotation
                double speed = MaxSpeed;
                double angular = MaxAngularRate;

                // If slow mode is on, reduce them
                // if (slowMode) {
                //     speed *= 0.10;    // 10% of normal speed
                //     angular *= 0.10; // 10% of normal turn rate
                // }

                // Read the raw joystick
                double rawX  = squareInput(driverJoystickLeft.getYAxis())  * speed;   // forward/back (note sign)
                double rawY  = squareInput(driverJoystickLeft.getXAxis())  * speed;   // strafe
                double rawRot = squareInput(-driverJoystickRight.getZRotation()) * angular; // rotation
                
                // Pass through the limiters
                double vx    = xSpeedLimiter.calculate(rawX);
                double vy    = ySpeedLimiter.calculate(rawY);
                double omega = rotLimiter.calculate(rawRot);
                
                SmartDashboard.putBoolean("Robot-Centric Drive", isRobotCentric);
                SmartDashboard.putBoolean("Slow Mode", slowMode);
                SmartDashboard.putBoolean("Track Tag", trackTag);

                // if (trackTag && vision.getTagIfInView(16).isPresent()) {
                //     // If we are tracking a tag, use the vision subsystem to get the angle to the tag
                //     // and rotate towards it.
                //     omega = -1.0 * vision.getTagIfInView(16).get().getYaw() * omega * .01;
                // }
                // Return the proper request
                if (isRobotCentric) {
                    // Robot-centric mode
                    return robotCentricDrive
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withRotationalRate(omega);
                } else {
                    // Field-centric mode
                    return fieldCentricDrive
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withRotationalRate(omega);
                }
            })
        );

        // Climber Bindings

        // Elevator Bindings
        operatorJoystickLeft.outerHatUp().whileTrue(elevator.moveUpCommand(20));
        operatorJoystickLeft.outerHatDown().whileTrue(elevator.moveDownCommand(20));

        operatorJoystickRight.outerHatUp().whileTrue(elevator.moveUpCommand(20));
        operatorJoystickRight.outerHatDown().whileTrue(elevator.moveDownCommand(20));

        operatorJoystickRight.f1Button().whileTrue(elevator.moveVariableCommand(operatorJoystickRight::getY));

        // EndEffector Bindings
        operatorJoystickLeft.triggerPrimary().onTrue(endEffector.autoIntakeAndHoldCommand(.5));
        operatorJoystickLeft.triggerSecondary().onTrue(endEffector.autoIntakeAndHoldCommand(.5));
        operatorJoystickLeft.redButton().whileTrue(endEffector.runIntakeContinuousCommand(-.5));
        operatorJoystickLeft.thumbButton().onTrue(endEffector.stopIntakeCommand());

        operatorJoystickRight.triggerPrimary().whileTrue(endEffector.runIntakeContinuousCommand(-.5));
        operatorJoystickRight.triggerSecondary().whileTrue(endEffector.runIntakeContinuousCommand(-.5));
        operatorJoystickRight.redButton().whileTrue(endEffector.runIntakeContinuousCommand(.5));
        operatorJoystickRight.thumbButton().onTrue(endEffector.stopIntakeCommand());

        // Pivot Bindings
        operatorJoystickLeft.innerHatUp().whileTrue(pivot.runPivotOut(.5));
        operatorJoystickLeft.innerHatDown().whileTrue(pivot.runPivotIn(.5));

        operatorJoystickRight.innerHatUp().whileTrue(pivot.runPivotOut(.5));
        operatorJoystickRight.innerHatDown().whileTrue(pivot.runPivotIn(.5));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Bind A button to run intake at full speed (1.0)
        // joystick.b().whileTrue(endEffector.runIntakeContinuousCommand(.25));
        // joystick.a().whileTrue(endEffector.runIntakeContinuousCommand(-.25));
        // joystick.x().whileTrue(endEffector.holdGamePieceCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
        // SignalLogger.enableAutoLogging(false);
    }

    public Command getAutonomousCommand() {
        // This method loads the auto when it is called, however, it is recommended
        // to first load your paths/autos when code starts, then return the
        // pre-loaded auto/path
        return autoChooser.getSelected();
    }

    /**
     * Squares the input value while preserving the sign.
     * For example, 0.5 becomes 0.25, and -0.5 becomes -0.25.
     */
    private double squareInput(double value) {
        return Math.copySign(value * value, value);
    }

}
