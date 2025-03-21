// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    // public final VisionSubsystem vision = new VisionSubsystem(drivetrain);

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
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
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
        .withVelocityX(MaxSpeed*0.25) // positive goes backwards
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
        pivot.setDefaultCommand(pivot.holdPivotAngleCommand()); // TEST ME       

        // ==================== Drivetrain Bindings ====================
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {

                    // Decide on your normal top speed/rotation
                    double speed = MaxSpeed;
                    double angular = MaxAngularRate;

                    // If slow mode is on, reduce them
                    if (Drivetrain.isRobotCentric) {
                        speed *= 0.05;    // 5% of normal speed
                        angular *= 0.50; // 50% of normal turn rate
                    } else if (Drivetrain.isSlowMode) {
                        speed *= 0.10;
                        angular *= 0.10;
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
        driverJoystickRight.rightButton().onTrue(
            new InstantCommand(drivetrain::seedFieldCentric));

        // // ==================== Climber Bindings ====================

        /*
         * OP RIGHT F2 - Prepare Climber
         */
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

        /*
         * OP RIGHT F3 - Climb
         */
        operatorJoystickRight.f3Button().onTrue(
            ratchet.lockRatchetCommand()
            .andThen(climber.rotateClimberToClimbedPositionCommand()).withTimeout(3.0));

        // ==================== Elevator Bindings ====================

        // Coral Positions
        operatorJoystickRight.lowerHatUp().onTrue(
            setPivotAndMoveElevatorCommand(PivotNew.ANGLE_CORAL_L4, Elevator.POSITION_CORAL_L4));
        operatorJoystickRight.lowerHatLeft().onTrue(
            setPivotAndMoveElevatorCommand(PivotNew.ANGLE_HARDSTOP, Elevator.POSITION_CORAL_L2));
        operatorJoystickRight.lowerHatRight().onTrue(
            setPivotAndMoveElevatorCommand(PivotNew.ANGLE_HARDSTOP, Elevator.POSITION_CORAL_L3));
        operatorJoystickRight.lowerHatDown().onTrue(
            setPivotAndMoveElevatorCommand(PivotNew.ANGLE_HARDSTOP, Elevator.POSITION_CORAL_L1));

        // Algae Positions
        operatorJoystickRight.innerHatUp().onTrue(
            setPivotAndMoveElevatorCommand(PivotNew.ANGLE_BARGE, Elevator.POSITION_ALGAE_BARGE));
        operatorJoystickRight.innerHatLeft().onTrue(
            setPivotAndMoveElevatorCommand(PivotNew.ANGLE_ALGAE_COLLECT, Elevator.POSITION_ALGAE_LOW));
        operatorJoystickRight.innerHatRight().onTrue(
            setPivotAndMoveElevatorCommand(PivotNew.ANGLE_ALGAE_COLLECT, Elevator.POSITION_ALGAE_HIGH));
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
            endEffector.intakeAlgaeCurrentLimitCommand()
            .andThen(endEffector.holdAlgaeCommand()
            ));

        endEffector.getCoralInnerDetectionTrigger().onTrue(
            endEffector.intakeCoralCommand());

        operatorJoystickRight.indexButon().onTrue(
            endEffector.stopIntakeCommand());

        // ==================== Pivot Bindings ====================
        operatorJoystickRight.outerHatLeft().onTrue( // TODO CHANGE TO JOG A FEW DEGREES
            pivot.jogPivotInCommand());
        operatorJoystickRight.outerHatRight().onTrue(
            pivot.jogPivotOutCommand());

        new Trigger(() -> endEffector.isHoldingAlgae() || endEffector.isHoldingCoral())
                .whileTrue(new RunCommand(() -> ledStrip.setColor(Color.kGreen), ledStrip));

        

        // // ==================== OPERATOR LEFT STICK DEBUG COMMANDS ====================

        /*
         * ==================== CLIMBER ====================
         * F2 - Rotate Climber back to starting position
         * F1 - Unlock Ratchet
         * F1 & Hold - Rotate Climber with Left Joystick
         * F3 - Lock Ratchet
         */
        operatorJoystickLeft.f1Button().whileTrue(
            climber.rotateClimberVariableCommad(operatorJoystickLeft::getYAxis));
        operatorJoystickLeft.f2Button().onTrue(
            climber.rotateClimberToStartingPositionCommand());
        operatorJoystickLeft.f1Button().onTrue(
            ratchet.unlockRatchetCommand());
        operatorJoystickLeft.f3Button().onTrue(
            ratchet.lockRatchetCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // This method loads the auto when it is called, however, it is recommended
        // to first load your paths/autos when code starts, then return the
        // pre-loaded auto/path
        return autoChooser.getSelected();
    }

    /**
     * move the elevator and pivot to positions to collect algae from the floor
     * and then auto trigger collecting algae when it is detected
     * and then move the pivot to the carry position
    //  */
    // private Command collectAlgaeFromGround() {
    //     return moveToCollectAlgaeFromGround()
    //             .andThen(endEffector.intakeAlgaeCurrentLimitCommand())
    //             .andThen(new ParallelCommandGroup(
    //                     endEffector.holdAlgaeCommand(),
    //                     pivot.setPositionAndHoldCommand(() -> Pivot.CARRY_ALGAE_POSITION_OFFSET),
    //                     new BlinkLEDCommand(ledStrip, Color.kGreen, 0.25)));
    // }

    /**
     * move the elevator and pivot to positions to collect algae
     */
    // private Command moveToCollectAlgaeFromGround() {
    //     return moveElevatorToPosition(Elevator.BOTTOM_POSITION, Pivot.CLEAR_RAMP)
    //             .andThen(pivot.setTargetPositionOffsetCommand(Pivot.COLLECT_ALGAE_POSITION_OFFSET));
    // }

    // private Command moveToCollectAlgaeFromReef(double position) {
    //     return moveElevatorToPosition(position, Pivot.CLEAR_RAMP)
    //             .andThen(pivot.setTargetPositionOffsetCommand(Pivot.COLLECT_ALGAE_POSITION_OFFSET));
    // }

    // private Command moveToScoreAlgae() {
    //     return moveElevatorToPosition(Elevator.TOP_POSITION, Pivot.CARRY_ALGAE_POSITION_OFFSET).andThen(pivot.setTargetPositionOffsetCommand(Pivot.SCORE_ALGAE_POSITION_OFFSET));
    // }

    /**
     * move the elevator and pivot to positions to collect coral
     */
    // private Command moveToCollectCoral() {
    //     return moveElevatorToPosition(Elevator.BOTTOM_POSITION, Pivot.CLEAR_RAMP)
    //             .andThen(pivot.setTargetPositionOffsetCommand(Pivot.COLLECT_CORAL_POSITION_OFFSET));
    // }

    // private Command moveToScoreCoral(double elevatorPosition, double pivotPosition) {
    //     return moveElevatorToPosition(elevatorPosition, Pivot.CLEAR_RAMP)
    //             .andThen(pivot.setTargetPositionOffsetCommand(pivotPosition));
    // }

    /**
     * move the end effector out of the way to the given position, then the elevator to the given height
     */
    // private Command moveElevatorToPosition(double elevatorPosition, double pivotPositionOffset) {
    //     // assume pivot is close enough if command is running for 1 second
    //     // a command can be cleared from the compound withTimeout Command with CommandScheduler.removeComposedCommand(Command)
    //     return pivot.setTargetPositionOffsetCommand(pivotPositionOffset).withTimeout(1.0)
    //             .andThen(new ParallelDeadlineGroup(
    //                     // assume elevator is close enough if command is running for 3 seconds
    //                     // and the PID is just struggling
    //                     elevator.moveToPositionCommand(elevatorPosition).withTimeout(3.0),
    //                     pivot.setPositionAndHoldCommand())
    //             );

    // }

    private Command setPivotAndMoveElevatorCommand(double pivotAngle, double elevatorPosition) {
        return pivot.setPivotAngleCommand(PivotNew.ANGLE_SAFE_MOVE).withTimeout(1.0)
                .andThen(new ParallelDeadlineGroup(
                    elevator.moveElevatorToPositionCommand(elevatorPosition).withTimeout(3.0),
                    pivot.holdPivotAngleCommand(PivotNew.ANGLE_SAFE_MOVE)
                ))
                .andThen(pivot.holdPivotAngleCommand(pivotAngle));
    }

}
