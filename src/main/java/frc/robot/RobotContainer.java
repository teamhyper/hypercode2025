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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ledCommands.BlinkLEDCommand;
import frc.robot.commands.ledCommands.SetLEDPatternCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.hyperlib.DriverInput;
import frc.robot.joysticks.ApemHF45Joystick;
import frc.robot.joysticks.VKBGladiatorJoystick;
import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    // Subsytem Initialization
    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    public final EndEffector endEffector = new EndEffector();
    public final Climber climber = new Climber();
    // public final VisionSubsystem vision = new VisionSubsystem();
    public final Elevator elevator = new Elevator();
    public final Pivot pivot = new Pivot();
    public final Ratchet ratchet = new Ratchet();
    public final Ramp ramp = new Ramp();
    public final LEDStrip ledStrip = new LEDStrip();
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

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {

        // Drivetrain Bindings
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
                    double rawX = driverJoystickLeft.getYAxis() * speed;   // forward/back (note sign)
                    double rawY = driverJoystickLeft.getXAxis() * speed;   // strafe
                    double rawRot = -driverJoystickRight.getZRotation() * angular; // rotation

                    // Pass through the limiters
                    double vx = DriverInput.filterAllowZero(rawX, xSpeedLimiter, rawX == 0);
                    double vy = DriverInput.filterAllowZero(rawY, ySpeedLimiter, rawY == 0);
                    double omega = DriverInput.filterAllowZero(rawRot, rotLimiter, rawRot == 0);

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

        // ==================== Climber Bindings ====================

        /*
         * OP RIGHT F2 - Prepare Climber
         */
        operatorJoystickRight.f2Button().onTrue(
                new ParallelDeadlineGroup(new ParallelCommandGroup(ramp.detachRampCommand(), ratchet.unlockRatchetCommand())
                        .andThen(climber.rotateClimberOutCommand()), new BlinkLEDCommand(ledStrip, Color.kYellow, 0.25))
                        .andThen(new BlinkLEDCommand(ledStrip, Color.kGreen, 0.25)));

        /*
         * OP RIGHT F3 - Climb
         */
        operatorJoystickRight.f3Button().onTrue(ratchet.lockRatchetCommand()
                .andThen(climber.rotateClimberInCommand()).andThen(new SetLEDPatternCommand(ledStrip)));

        // ==================== Elevator Bindings ====================

        // Coral Positions
        operatorJoystickRight.lowerHatUp()
                .onTrue(moveToScoreAlgae());
        operatorJoystickRight.lowerHatLeft().or(operatorJoystickRight.lowerHatRight())
                .onTrue(elevator.moveToPositionCommand(0));
        operatorJoystickRight.lowerHatDown().onTrue(elevator.moveToPositionCommand(0));

        // Algae Positions -- TODO add pivot commands to set the correct angle
        operatorJoystickRight.innerHatUp()
                .onTrue(elevator.moveToPositionCommand(0));
        operatorJoystickRight.innerHatLeft().or(operatorJoystickRight.innerHatRight())
                .onTrue(elevator.moveToPositionCommand(0));
        operatorJoystickRight.innerHatDown().onTrue(elevator.moveToPositionCommand(0));

        // Elevator to floor
        operatorJoystickRight.thumbButton().onTrue(moveElevatorToBottom());

        // Manual Elevator Commands
        operatorJoystickRight.outerHatUp().whileTrue(elevator.moveUpCommand(20)); //TODO set this to pass position jog up/down 1 inch
        operatorJoystickRight.outerHatDown().whileTrue(elevator.moveDownCommand(20));

        // Pull BACK on the joystick to move the elevator up
        operatorJoystickRight.pinkyButton().whileTrue(elevator.moveVariableCommand(operatorJoystickRight::getY));

        // ==================== EndEffector Bindings ====================
        operatorJoystickRight.triggerPrimary()
                .onTrue(endEffector.scoreGamePieceCommand().andThen(new BlinkLEDCommand(ledStrip, Color.kRed, 0.25)));

        operatorJoystickRight.redButton() // TODO add pivot command to set pivot to correct position?
                .onTrue(collectAlgae());

        endEffector.getCoralInnerDetectionTrigger()
                .onTrue(new SequentialCommandGroup(endEffector.intakeCoralCommand())
                        .andThen(new BlinkLEDCommand(ledStrip, Color.kGreen, 0.25)));

        operatorJoystickRight.indexButon().onTrue(endEffector.stopIntakeCommand());

        // ==================== Pivot Bindings ====================
        operatorJoystickRight.innerHatUp().whileTrue(pivot.runPivotOutAtSpeed(.15));
        operatorJoystickRight.innerHatDown().whileTrue(pivot.runPivotInAtSpeed(.15));
        operatorJoystickLeft.innerHatLeft().onTrue(pivot.setTargetPositionCommand(() -> Pivot.ALL_IN_POSITION));
        operatorJoystickLeft.innerHatDown().onTrue(pivot.setTargetPositionOffsetCommand(Pivot.COLLECT_CORAL_POSITION_OFFSET));
        operatorJoystickLeft.innerHatRight().onTrue(pivot.setTargetPositionCommand(() -> 60));
        operatorJoystickLeft.innerHatUp().onTrue(pivot.setTargetPositionOffsetCommand(Pivot.CARRY_ALGAE_POSITION_OFFSET));

        // LED Bindings
        new Trigger(RobotState::isEnabled)
                .onTrue(new BlinkLEDCommand(ledStrip, Color.kRed, 0.25));

        // TEST COMMANDS

        operatorJoystickLeft.f1Button().whileTrue(climber.rotateClimberVariableCommad(operatorJoystickLeft::getYAxis));
        operatorJoystickLeft.f2Button().onTrue(climber.rotateClimberToStartingPositionCommand());

        operatorJoystickLeft.f1Button().onTrue(ratchet.unlockRatchetCommand());
        operatorJoystickLeft.f3Button().onTrue(ratchet.lockRatchetCommand());

        operatorJoystickLeft.lowerHatUp().onTrue(elevator.moveToPositionCommand(50));
        operatorJoystickLeft.lowerHatDown().onTrue(elevator.moveToPositionCommand(30));
        operatorJoystickLeft.thumbButton().onTrue(moveToCollectCoral());

        // operatorJoystickLeft.triggerPrimary().whileTrue(elevator.testGravityCommand());

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
     * move the elevator and pivot to positions to collect algae from the floor
     * and then auto trigger collecting algae when it is detected
     * and then move the pivot to the carry position
     */
    private Command collectAlgae() {
        return moveToCollectAlgaeFromGround()
                .andThen(endEffector.intakeAlgaeCurrentLimitCommand())
                .andThen(new ParallelDeadlineGroup(
                        endEffector.holdAlgaeCommand(),
                        pivot.setPositionAndHoldCommand(() -> Pivot.CARRY_ALGAE_POSITION_OFFSET),
                        new BlinkLEDCommand(ledStrip, Color.kGreen, 0.25)));
    }

    /**
     * move the elevator and pivot to positions to collect algae
     */
    private Command moveToCollectAlgaeFromGround() {
        return moveElevatorToBottom()
                .andThen(pivot.setTargetPositionOffsetCommand(Pivot.COLLECT_ALGAE_POSITION_OFFSET));
    }

    private Command moveToScoreAlgae() {
        return moveElevatorToPosition(() -> Elevator.TOP_POSITION).andThen(pivot.setTargetPositionOffsetCommand(Pivot.SCORE_ALGAE_POSITION_OFFSET));
    }

    /**
     * move the elevator and pivot to positions to collect coral
     */
    private Command moveToCollectCoral() {
        return moveElevatorToBottom()
                .andThen(pivot.setTargetPositionOffsetCommand(Pivot.COLLECT_CORAL_POSITION_OFFSET));
    }

    /**
     * move the end effector out of the way, then the elevator to the bottom
     */
    private Command moveElevatorToBottom() {
        return pivot.setTargetPositionOffsetCommand(Pivot.SCORE_CORAL_POSITION_OFFSET).andThen(elevator.moveToPositionCommand(Elevator.BOTTOM_POSITION));
    }

    /**
     * move the end effector out of the way, then the elevator to the given height
     */
    private Command moveElevatorToPosition(DoubleSupplier elevatorPosition) {
        return pivot.setTargetPositionOffsetCommand(Pivot.CARRY_ALGAE_POSITION_OFFSET).andThen(elevator.moveToPositionCommand(elevatorPosition.getAsDouble()));
    }

    /**
     * Squares the input value while preserving the sign.
     * For example, 0.5 becomes 0.25, and -0.5 becomes -0.25.
     */
    private double squareInput(double value) {
        return Math.copySign(value * value, value);
    }

}
