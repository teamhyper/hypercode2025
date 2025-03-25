// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.hyperlib.DriverInput;
import frc.robot.joysticks.ApemHF45Joystick;
import frc.robot.joysticks.VKBGladiatorJoystick;
import frc.robot.subsystems.*;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    // Subsytem Initialization
    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    public final EndEffector endEffector = new EndEffector();
    public final Climber climber = new Climber();
    public final Elevator elevator = new Elevator();
    public final PivotNew pivot = new PivotNew();
    public final Ratchet ratchet = new Ratchet();
    public final Ramp ramp = new Ramp();
    public final LEDStrip ledStrip = new LEDStrip();
    // public final VisionSubsystem vision = new VisionSubsystem();
    public final Vision vision = new Vision();

    // Slew Rate Limiter
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(Constants.xSpeedLimiter);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(Constants.ySpeedLimiter);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.rotLimiter);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SendableChooser<Command> autoChooser;

    // Controller Initialization
    ApemHF45Joystick driverJoystickLeft = new ApemHF45Joystick(0);
    ApemHF45Joystick driverJoystickRight = new ApemHF45Joystick(1);
    VKBGladiatorJoystick operatorJoystickLeft = new VKBGladiatorJoystick(2);
    VKBGladiatorJoystick operatorJoystickRight = new VKBGladiatorJoystick(3);

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry logger = new Telemetry(MaxSpeed);
    
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

    public RobotContainer() {
        configureBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        autoChooser.addOption("Drive Off Line", drivetrain.applyRequest(() -> robotCentricDrive
        .withVelocityX(MaxSpeed*0.25) // positive goes backwards TODO FIX THE FLIPPED ODOMETRY???
        .withVelocityY(0)
        .withRotationalRate(0))
                .withTimeout(1.0)
                .andThen(drivetrain.applyRequest(() -> robotCentricDrive
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                ));

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {

        elevator.setDefaultCommand(elevator.holdElevatorPositionCommand());
        ledStrip.setDefaultCommand(new InstantCommand(() -> ledStrip.setColor(Color.kRed), ledStrip));
        
        // ==================== Drivetrain Bindings ====================
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {

                    // Decide on your normal top speed/rotation
                    double speed = MaxSpeed;
                    double angular = MaxAngularRate;

                    // If slow mode is on, reduce them
                    if (Drivetrain.isRobotCentric) {
                        speed *= 0.1;
                        angular *= 0.25; 
                    } else if (Drivetrain.isSlowMode) {
                        speed *= 0.20;
                        angular *= 0.20;
                    } else if (elevator.getPosition() > Elevator.STAGE_1) {
                        speed *= 0.20;
                        angular *= 0.20;
                    }

                    // Read the raw joystick
                    double rawX = driverJoystickLeft.getYAxis() * speed;   // forward/back (note sign)
                    double rawY = driverJoystickLeft.getXAxis() * speed;   // strafe
                    double rawRot = -driverJoystickRight.getZRotation() * angular; // rotation

                    // Pass through the limiters
                    double vx = DriverInput.filterAllowZero(rawX, xSpeedLimiter, rawX == 0);
                    double vy = DriverInput.filterAllowZero(rawY, ySpeedLimiter, rawY == 0);
                    double omega = DriverInput.filterAllowZero(rawRot, rotLimiter, rawRot == 0);                  

                    // Return the proper request
                    if (Drivetrain.isRobotCentric) {
                        // Robot-centric mode
                        return robotCentricDrive
                                .withVelocityX(-vx)
                                .withVelocityY(-vy)
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

        // ==================== Drivetrain Bindings ====================
        driverJoystickLeft.leftButton().onTrue(
            new InstantCommand(() -> Drivetrain.isRobotCentric = !Drivetrain.isRobotCentric));

        driverJoystickLeft.rightButton().onTrue(
            new InstantCommand( () -> Drivetrain.isSlowMode = !Drivetrain.isSlowMode));

        // driverJoystickRight.leftButton().whileTrue(getAutonomousCommand());

        driverJoystickRight.rightButton().onTrue(
            new InstantCommand(drivetrain::seedFieldCentric));

        // ==================== Climber Bindings ====================

        operatorJoystickRight.f2Button().onTrue(
                    new ParallelDeadlineGroup(
                        ramp.detachRampCommand(), 
                        ratchet.unlockRatchetCommand(), 
                        pivot.setPivotAngleCommand(PivotNew.ANGLE_ALGAE_COLLECT))
                    .andThen(new ParallelDeadlineGroup(
                        climber.rotateClimberToStartingPositionCommand(),
                        pivot.holdPivotAngleCommand(PivotNew.ANGLE_ALGAE_COLLECT)
                    )).andThen(new ParallelCommandGroup(
                        pivot.holdPivotAngleCommand(PivotNew.ANGLE_ALGAE_COLLECT),
                        new RunCommand(() -> ledStrip.setColor(Color.kGreen))
                    )));

        operatorJoystickRight.f3Button().onTrue(
            ratchet.lockRatchetCommand()
            .andThen(climber.rotateClimberToClimbedPositionCommand()).withTimeout(3.0));

        // ==================== Elevator Bindings ====================

        // Coral Positions
        operatorJoystickRight.lowerHatUp().onTrue(
            setPivotAndMoveElevatorCommand(PivotNew.ANGLE_CORAL_L4, Elevator.POSITION_CORAL_L4));
        operatorJoystickRight.lowerHatRight().onTrue(
            setPivotAndMoveElevatorCommand(PivotNew.ANGLE_HARDSTOP, Elevator.POSITION_CORAL_L3));
        operatorJoystickRight.lowerHatLeft().onTrue(
            setPivotAndMoveElevatorCommand(PivotNew.ANGLE_HARDSTOP, Elevator.POSITION_CORAL_L2));        
        operatorJoystickRight.lowerHatDown().onTrue(
            setPivotAndMoveElevatorCommand(PivotNew.ANGLE_HARDSTOP, Elevator.POSITION_CORAL_L1));

        // Algae Positions
        operatorJoystickRight.innerHatUp().onTrue(
            setPivotAndMoveElevatorCommand(PivotNew.ANGLE_BARGE, Elevator.POSITION_ALGAE_BARGE));
        operatorJoystickRight.innerHatRight().onTrue(
            setPivotAndMoveElevatorCommand(PivotNew.ANGLE_ALGAE_COLLECT, Elevator.POSITION_ALGAE_HIGH));
        operatorJoystickRight.innerHatLeft().onTrue(
            setPivotAndMoveElevatorCommand(PivotNew.ANGLE_ALGAE_COLLECT, Elevator.POSITION_ALGAE_LOW));
        operatorJoystickRight.innerHatDown().onTrue(
            setPivotAndMoveElevatorCommand(PivotNew.ANGLE_ALGAE_COLLECT, Elevator.POSITION_ALGAE_GROUND));

        // Floor Position
        operatorJoystickRight.thumbButton().onTrue(
            setPivotAndMoveElevatorCommand(PivotNew.ANGLE_HARDSTOP, Elevator.BOTTOM_POSITION));

        // Manual Elevator Jogging
        operatorJoystickRight.outerHatUp().onTrue(
            elevator.jogElevatorUpCommand(2));
        operatorJoystickRight.outerHatDown().onTrue(
            elevator.jogElevatorDownCommand(2));
        operatorJoystickRight.pinkyButton().whileTrue(
            // Pull BACK on the joystick to move the elevator up
            elevator.moveElevatorVariableCommand(operatorJoystickRight::getY));

        // ==================== EndEffector Bindings ====================

        operatorJoystickRight.triggerPrimary().onTrue(
            endEffector.scoreGamePieceCommand());

        operatorJoystickRight.redButton().onTrue(
            endEffector.intakeAlgaeCommand()
            .andThen(endEffector.holdAlgaeCommand()
            .until(() -> !endEffector.isHoldingAlgae()
            )));

        endEffector.getCoralInnerDetectionTrigger().whileTrue(
            endEffector.intakeCoralCommand());

        operatorJoystickRight.indexButon().onTrue(
            endEffector.stopIntakeCommand());

        // ==================== Pivot Bindings ====================
        operatorJoystickRight.outerHatLeft().whileTrue( // TODO CHANGE TO JOG A FEW DEGREES
            pivot.runPivotCommand(-.15));

        operatorJoystickRight.outerHatRight().whileTrue(
            pivot.runPivotCommand(.15));

        operatorJoystickRight.f1Button().whileTrue(pivot.runPivotVariableCommand(operatorJoystickRight::getY));

        
        // ==================== LED Triggers ====================
        new Trigger(() -> (endEffector.isHoldingAlgae() || endEffector.isHoldingCoral()) 
                            && !(Drivetrain.isRobotCentric || Drivetrain.isSlowMode))
            .whileTrue(new RunCommand(() -> {
                if (endEffector.isDetectingReef() && endEffector.isHoldingCoral()) {
                    ledStrip.setColor(Color.kBlue);
                } else {
                    ledStrip.setColor(Color.kGreen);
                }
            }, ledStrip));

        new Trigger(() -> (endEffector.isHoldingAlgae() || endEffector.isHoldingCoral()) 
                            && (Drivetrain.isRobotCentric || Drivetrain.isSlowMode))
            .whileTrue(new ConditionalCommand(
                ledStrip.setColorAndBlinkCommand(Color.kBlue), 
                ledStrip.setColorAndBlinkCommand(Color.kGreen), 
                () -> (endEffector.isDetectingReef() && endEffector.isHoldingCoral())));
                
        new Trigger(() -> (endEffector.isHoldingAlgae() || endEffector.isHoldingCoral()) 
                            && (Drivetrain.isRobotCentric || Drivetrain.isSlowMode)
                            && (endEffector.isDetectingReef() && endEffector.isHoldingCoral()))
                .whileTrue(ledStrip.setColorAndBlinkCommand(Color.kBlue));

        new Trigger(() -> (endEffector.isHoldingAlgae() || endEffector.isHoldingCoral()) 
                            && (Drivetrain.isRobotCentric || Drivetrain.isSlowMode)
                            && !(endEffector.isDetectingReef() && endEffector.isHoldingCoral()))
                .whileTrue(ledStrip.setColorAndBlinkCommand(Color.kGreen));

        new Trigger(() -> !(endEffector.isHoldingAlgae() || endEffector.isHoldingCoral()) 
                            && (Drivetrain.isRobotCentric || Drivetrain.isSlowMode))
            .whileTrue(ledStrip.setColorAndBlinkCommand(Color.kRed));

        // ==================== OPERATOR LEFT STICK DEBUG COMMANDS ====================

        operatorJoystickLeft.triggerPrimary().whileTrue(endEffector.runIntakeCommand(1.0));
        operatorJoystickLeft.indexButon().whileTrue(endEffector.runIntakeCommand(-1.0));

        operatorJoystickLeft.lowerHatDown().whileTrue(pivot.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        operatorJoystickLeft.lowerHatLeft().whileTrue(pivot.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        operatorJoystickLeft.lowerHatUp().whileTrue(pivot.sysIdDynamic(SysIdRoutine.Direction.kForward));
        operatorJoystickLeft.lowerHatRight().whileTrue(pivot.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Climber
        // operatorJoystickLeft.f2Button().onTrue(
        //     climber.rotateClimberToStartingPositionCommand());
        // operatorJoystickLeft.f1Button().whileTrue(
        //     climber.rotateClimberVariableCommad(operatorJoystickLeft::getYAxis));        
        // operatorJoystickLeft.f1Button().onTrue(
        //     ratchet.unlockRatchetCommand());
        // operatorJoystickLeft.f3Button().onTrue(
        //     ratchet.lockRatchetCommand());
        // operatorJoystickLeft.sw1Up().or(operatorJoystickLeft.sw1Down()).onTrue(
        //     ramp.detachRampCommand());
            
        operatorJoystickLeft.f2Button().whileTrue(pivot.holdPivotAngleCommand(PivotNew.ANGLE_HARDSTOP));
        operatorJoystickLeft.f1Button().whileTrue(pivot.holdPivotAngleCommand(PivotNew.ANGLE_SAFE_MOVE));
        operatorJoystickLeft.f3Button().whileTrue(pivot.holdPivotAngleCommand(PivotNew.ANGLE_CORAL_L4));
        operatorJoystickLeft.sw1Up().whileTrue(pivot.holdPivotAngleCommand(PivotNew.ANGLE_ALGAE_COLLECT));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // This method loads the auto when it is called, however, it is recommended
        // to first load your paths/autos when code starts, then return the
        // pre-loaded auto/path
        return autoChooser.getSelected();
    }

    // ==================== COMPOUND COMMANDS ====================

    private Command setPivotAndMoveElevatorCommand(double pivotAngle, double elevatorPosition) {
        return pivot.setPivotAngleCommand(PivotNew.ANGLE_SAFE_MOVE)
                .andThen(new ParallelDeadlineGroup(
                    elevator.moveElevatorToPositionCommand(elevatorPosition),
                    // .withTimeout(4.0),
                    pivot.holdPivotAngleCommand(PivotNew.ANGLE_SAFE_MOVE)
                ))
                .andThen(pivot.holdPivotAngleCommand(pivotAngle));
    }

}
