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
import frc.robot.commands.ledCommands.BlinkLEDCommand;
import frc.robot.commands.ledCommands.SetLEDPatternCommand;
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
    // public final VisionSubsystem vision = new VisionSubsystem(drivetrain);
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

        /*
         * ==================== DRIVER MODES ====================
         * LEFT JOYSTICK LEFT - FIELD CENTRIC MODE
         * LEFT JOYSTICK RIGHT - ROBOT CENTRIC + SLOW MODE
         * RIGHT JOYSTICK LEFT - 
         * RIGHT JOYSTICK RIGHT - RESET FIELD CENTRIC HEADING
         */

        driverJoystickLeft.leftButton().onTrue(
            new InstantCommand(() -> Drivetrain.isRobotCentric = false));
        driverJoystickLeft.rightButton().onTrue(
            new InstantCommand(() -> Drivetrain.isRobotCentric = true));
        driverJoystickLeft.rightButton().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // ==================== Climber Bindings ====================

        /*
         * OP RIGHT F2 - Prepare Climber
         */
        operatorJoystickRight.f2Button().onTrue(
                new ParallelDeadlineGroup(
                    new ParallelDeadlineGroup(
                        ramp.detachRampCommand(), 
                        ratchet.unlockRatchetCommand(), 
                        pivot.setTargetPositionOffsetCommand(Pivot.COLLECT_ALGAE_POSITION_OFFSET)) // TODO TEST
                    .andThen(climber.rotateClimberOutCommand()), new BlinkLEDCommand(ledStrip, Color.kYellow, 0.25))
                .andThen(new BlinkLEDCommand(ledStrip, Color.kGreen, 0.25)));

        /*
         * OP RIGHT F3 - Climb
         */
        operatorJoystickRight.f3Button().onTrue(
            ratchet.lockRatchetCommand()
            .andThen(climber.rotateClimberInCommand()).withTimeout(3.0) // TODO TEST
            .andThen(new SetLEDPatternCommand(ledStrip)));

        // ==================== Elevator Bindings ====================

        // Coral Positions
        operatorJoystickRight.lowerHatUp().onTrue(
            moveToScoreCoral(Elevator.POSITION_CORAL_L4, Pivot.SCORE_CORAL_L4_POSITION_OFFSET));
        operatorJoystickRight.lowerHatLeft().onTrue(
            moveToScoreCoral(Elevator.POSITION_CORAL_L2, Pivot.SCORE_CORAL_L2L3_POSITION_OFFSET));
        operatorJoystickRight.lowerHatRight().onTrue(
            moveToScoreCoral(Elevator.POSITION_CORAL_L3, Pivot.SCORE_CORAL_L2L3_POSITION_OFFSET));
        operatorJoystickRight.lowerHatDown().onTrue(
            moveToScoreCoral(Elevator.POSITION_CORAL_L1, Pivot.SCORE_CORAL_L1_POSITION_OFFSET));

        // Algae Positions
        operatorJoystickRight.innerHatUp().onTrue(
            moveToScoreAlgae());
        operatorJoystickRight.innerHatLeft().or(operatorJoystickRight.innerHatRight()).onTrue(
            moveToCollectAlgaeFromReef(Elevator.POSITION_ALGAE_HIGH));
        operatorJoystickRight.innerHatDown().onTrue(
            moveToCollectAlgaeFromReef(Elevator.POSITION_ALGAE_LOW));

        // Floor Position
        operatorJoystickRight.thumbButton().onTrue(
            moveToCollectCoral());

        // Manual Elevator Jogging
        operatorJoystickRight.outerHatUp().onTrue( // TODO TEST
            elevator.moveUpCommand(2));
        operatorJoystickRight.outerHatDown().onTrue(
            elevator.moveDownCommand(2));
        // Pull BACK on the joystick to move the elevator up
        operatorJoystickRight.pinkyButton().whileTrue(
            elevator.moveVariableCommand(operatorJoystickRight::getY));

        // ==================== EndEffector Bindings ====================

        operatorJoystickRight.triggerPrimary().onTrue(
            endEffector.scoreGamePieceCommand()
            .andThen(new BlinkLEDCommand(ledStrip, Color.kRed, 0.25)).withTimeout(1.0)
            .andThen(new InstantCommand(() -> ledStrip.setColor(Color.kRed))));

        operatorJoystickRight.redButton().onTrue(
            collectAlgaeFromGround()
            .andThen(new BlinkLEDCommand(ledStrip, Color.kGreen, 0.25)).withTimeout(1.0)
            .andThen(new InstantCommand(() -> ledStrip.setColor(Color.kGreen))));

        endEffector.getCoralInnerDetectionTrigger().onTrue(
            endEffector.intakeCoralCommand()
            .andThen(new BlinkLEDCommand(ledStrip, Color.kGreen, 0.25)).withTimeout(1.0)
            .andThen(new InstantCommand(() -> ledStrip.setColor(Color.kGreen))));

        operatorJoystickRight.indexButon().onTrue(
            endEffector.stopIntakeCommand());

        // ==================== Pivot Bindings ====================
        operatorJoystickRight.outerHatLeft().whileTrue( // TODO CHANGE TO JOG A FEW DEGREES
            pivot.runPivotOutAtSpeed(.15));
        operatorJoystickRight.outerHatRight().whileTrue(
            pivot.runPivotInAtSpeed(.15));

        

        // ==================== OPERATOR LEFT STICK DEBUG COMMANDS ====================

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

        /*
         * ==================== ELEVATOR ====================
         * LOWER HAT UP - POSITION_CORAL_L4
         * LOWER HAT LEFT - POSITION_CORAL_L2
         * LOWER HAT RIGHT - POSITION_CORAL_L3
         * LOWER HAT DOWN - POSITION_CORAL_L1
         */
        operatorJoystickLeft.lowerHatUp().onTrue(
            elevator.moveToPositionCommand(Elevator.POSITION_CORAL_L4).withTimeout(3.0));
        operatorJoystickLeft.lowerHatLeft().onTrue(
            elevator.moveToPositionCommand(Elevator.POSITION_CORAL_L2).withTimeout(3.0));
        operatorJoystickLeft.lowerHatRight().onTrue(
            elevator.moveToPositionCommand(Elevator.POSITION_CORAL_L3).withTimeout(3.0));
        operatorJoystickLeft.lowerHatDown().onTrue(
            elevator.moveToPositionCommand(Elevator.POSITION_CORAL_L1).withTimeout(3.0));

        /*
         * Inner HAT UP - POSITION_ALGAE_BARGE
         * Inner HAT LEFT - POSITION_ALGAE_LOW
         * Inner HAT RIGHT - POSITION_ALGAE_HIGH
         * Inner HAT DOWN - POSITION_ALGAE_GROUND
         */
        operatorJoystickLeft.innerHatUp().onTrue(
            elevator.moveToPositionCommand(Elevator.POSITION_ALGAE_BARGE).withTimeout(3.0));
        operatorJoystickLeft.innerHatLeft().onTrue(
            elevator.moveToPositionCommand(Elevator.POSITION_ALGAE_LOW).withTimeout(3.0));
        operatorJoystickLeft.innerHatRight().onTrue(
            elevator.moveToPositionCommand(Elevator.POSITION_ALGAE_HIGH).withTimeout(3.0));
        operatorJoystickLeft.innerHatDown().onTrue(
            elevator.moveToPositionCommand(Elevator.POSITION_ALGAE_GROUND).withTimeout(3.0));

        operatorJoystickLeft.thumbButton().onTrue(
            elevator.moveToPositionCommand(Elevator.BOTTOM_POSITION));

        /*
         * ==================== ENDEFFECTOR ====================
         * Trigger - Intake Algae, Intake Coral, Score Coral
         * Index - Score Algae, Reverse Coral
         */

         operatorJoystickLeft.triggerPrimary().whileTrue(
            endEffector.runIntakeCommand(1.0));
        operatorJoystickLeft.indexButon().whileTrue(
            endEffector.runIntakeCommand(-1.0));


        /*
         * ==================== PIVOT ====================
         * OUTER HAT UP - 
         * OUTER HAT LEFT - 
         * OUTER HAT RIGHT - 
         * OUTER HAT DOWN - 
         */

        operatorJoystickLeft.outerHatLeft().onTrue(
            pivot.setTargetPositionCommand(() -> Pivot.ALL_IN_POSITION));
        operatorJoystickLeft.outerHatDown().onTrue(
            pivot.setTargetPositionOffsetCommand(0));
        operatorJoystickLeft.outerHatRight().onTrue(
            pivot.setTargetPositionOffsetCommand(Pivot.COLLECT_ALGAE_POSITION_OFFSET));
        operatorJoystickLeft.outerHatUp().onTrue(
            pivot.setTargetPositionOffsetCommand(Pivot.CARRY_ALGAE_POSITION_OFFSET));        

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
     */
    private Command collectAlgaeFromGround() {
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
        return moveElevatorToPosition(Elevator.BOTTOM_POSITION, Pivot.CLEAR_RAMP)
                .andThen(pivot.setTargetPositionOffsetCommand(Pivot.COLLECT_ALGAE_POSITION_OFFSET));
    }

    private Command moveToCollectAlgaeFromReef(double position) {
        return moveElevatorToPosition(position, Pivot.CLEAR_RAMP)
                .andThen(pivot.setTargetPositionOffsetCommand(Pivot.COLLECT_ALGAE_POSITION_OFFSET));
    }

    private Command moveToScoreAlgae() {
        return moveElevatorToPosition(Elevator.TOP_POSITION, Pivot.CARRY_ALGAE_POSITION_OFFSET).andThen(pivot.setTargetPositionOffsetCommand(Pivot.SCORE_ALGAE_POSITION_OFFSET));
    }

    /**
     * move the elevator and pivot to positions to collect coral
     */
    private Command moveToCollectCoral() {
        return moveElevatorToPosition(Elevator.BOTTOM_POSITION, Pivot.CLEAR_RAMP)
                .andThen(pivot.setTargetPositionOffsetCommand(Pivot.COLLECT_CORAL_POSITION_OFFSET));
    }

    private Command moveToScoreCoral(double elevatorPosition, double pivotPosition) {
        return moveElevatorToPosition(elevatorPosition, Pivot.CLEAR_RAMP)
                .andThen(pivot.setTargetPositionOffsetCommand(pivotPosition));
    }

    /**
     * move the end effector out of the way to the given position, then the elevator to the given height
     */
    private Command moveElevatorToPosition(double elevatorPosition, double pivotPositionOffset) {
        return pivot.setTargetPositionOffsetCommand(pivotPositionOffset)
                .andThen(new ParallelDeadlineGroup(
                        elevator.moveToPositionCommand(elevatorPosition),
                        pivot.setPositionAndHoldCommand())
                );

    }

    /**
     * Squares the input value while preserving the sign.
     * For example, 0.5 becomes 0.25, and -0.5 becomes -0.25.
     */
    private double squareInput(double value) {
        return Math.copySign(value * value, value);
    }

}
