package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class Pivot extends SubsystemBase {
    // TODO: get positions
    // SCORE_CORAL_POSITION_OFFSET is a guesstimate of the midpoint (45deg) between the ALL_IN and ALL_OUT positions
    public static final double ALL_IN_POSITION = 211.5, ALL_OUT_POSITION = 304;
    public static final double COLLECT_CORAL_POSITION_OFFSET = 41.0, COLLECT_ALGAE_POSITION_OFFSET = ALL_OUT_POSITION - ALL_IN_POSITION, SCORE_ALGAE_POSITION_OFFSET = 0.0, SCORE_CORAL_POSITION_OFFSET = 48.0, CARRY_ALGAE_POSITION_OFFSET = 60.0;

    // PID coefficients
    // from REV example p = 0.1, i = 1e-4, d = 1
    private static final double P = 0.0125, I = 0.0, D = 0.01;
    private static final double aFF = 0.01, MIN_OUTPUT = -0.1, MAX_OUTPUT = 0.225;
    // Max Motion parameters
    private static final double MAX_VELOCITY = 3000.0, MAX_ACCELERATION = MAX_VELOCITY * 100000, ALLOWED_ERROR = 2.5;
    // REV soft limit parameters
    private static final boolean FORWARD_SOFT_LIMIT_ENABLED = true, REVERSE_SOFT_LIMIT_ENABLED = true;
    private static final int PIVOT_MOTOR_ID = 20;
    private static final double G = 1.51, V = 0.78, A = 0.07, S = 0.0;
    private final SparkMax motor;
    private final SparkAbsoluteEncoder encoder;
    private final SparkClosedLoopController pidController;
    private final ArmFeedforward pivotFeedforward = new ArmFeedforward(S, G, V, A);
    private final ClosedLoopSlot pidSlotMain = ClosedLoopSlot.kSlot0;
    private double target;


    public Pivot() {
        this(PIVOT_MOTOR_ID);
    }

    public Pivot(int motorId) {
        motor = new SparkMax(motorId, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();

        config
                .inverted(false)
                .idleMode(IdleMode.kBrake);

        config.absoluteEncoder
                .inverted(true)
                .positionConversionFactor(360);

        // TODO: tune PID values
        config.closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
                .pid(P, I, D, pidSlotMain)
                .outputRange(MIN_OUTPUT, MAX_OUTPUT);

        // TODO: tune maxMotion values
        config.closedLoop.maxMotion
                .maxVelocity(MAX_VELOCITY, pidSlotMain)
                .maxAcceleration(MAX_ACCELERATION, pidSlotMain)
                .allowedClosedLoopError(ALLOWED_ERROR, pidSlotMain);

        config.softLimit
                .forwardSoftLimitEnabled(FORWARD_SOFT_LIMIT_ENABLED)
                .reverseSoftLimitEnabled(REVERSE_SOFT_LIMIT_ENABLED)
                .reverseSoftLimit(ALL_IN_POSITION)
                .forwardSoftLimit(ALL_OUT_POSITION);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController = motor.getClosedLoopController();
        encoder = motor.getAbsoluteEncoder();

        setPosition(encoder::getPosition);

        // place the subsystem on SmartDashboard
        SmartDashboard.putData(this);
        setDefaultCommand(setPositionAndHoldCommand());
    }

    /**
     * Method to set the pivot position using Max Motion Profiling
     *
     * @param position target value for the PID
     */
    public void setPosition(DoubleSupplier position) {
        this.target = position.getAsDouble();

        System.out.printf("Pivot.setPosition(): Position: %.2f%n", this.target);
//        pidController.setReference(this.target, SparkMax.ControlType.kMAXMotionPositionControl, pidSlotMain, aFF);
        pidController.setReference(this.target, SparkMax.ControlType.kPosition, pidSlotMain, aFF);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("On Target", this::onTarget, null);
        builder.addDoubleProperty("Target", this::getTarget, null);
        builder.addDoubleProperty("EncoderPosition (Actual)", this::getPosition, null);
        builder.addDoubleProperty("EncoderPosition (Offset)", () -> getPosition() - ALL_IN_POSITION, null);
        builder.addBooleanProperty("CanMove In", () -> getPosition() > ALL_IN_POSITION, null);
        builder.addBooleanProperty("CanMove Out", () -> getPosition() < ALL_OUT_POSITION, null);
        builder.addDoubleProperty("Speed", motor::get, null);
        builder.addDoubleProperty("Applied Output", motor::getAppliedOutput, null);
    }


    /**
     * Method to get the current position
     *
     * @return - current position of the absolute encoder
     */
    public double getPosition() {
        return encoder.getPosition();
    }

    /**
     * sets the PID setpoint to the current position and then runs continuously so the closed loop PID on
     * the SparkMax holds to that position
     *
     * @return command that sets the PID setpoint to the current position and holds
     */
    public Command setPositionAndHoldCommand() {
        return setTargetPositionCommand(encoder::getPosition, true);
    }

    /**
     * stop the motor (speed 0)
     */
    public void stop() {
        motor.stopMotor();
    }

    /**
     * @return the stored target value
     */
    private double getTarget() {
        return this.target;
    }

    /**
     * is the Pivot encoder position at the given target
     *
     * @param target the specific target position to check against
     * @return if the Pivot is at the given target
     */
    public boolean onTarget(DoubleSupplier target) {
        return Math.abs(encoder.getPosition() - target.getAsDouble()) <= ALLOWED_ERROR;
    }

    /**
     * is the Pivot encoder position at the previously set target
     *
     * @return if the Pivot is at the set target
     */
    public boolean onTarget() {
        return this.onTarget(this::getTarget);
    }

    /**
     * Command to move pivot manually outward while a button is held
     *
     * @param speed absolute value of speed to run the pivot motor outwards at
     * @return command to run the motor outward at a speed
     */
    public Command runPivotOutAtSpeed(double speed) {
        return new RunCommand(() -> motor.set(speed), this).finallyDo(interrupted -> stop());
    }

    /**
     * Command to move pivot manually inward while a button is held
     *
     * @param speed absolute value of speed to run the pivot motor inwards at
     * @return command to run the motor inward at a speed
     */
    public Command runPivotInAtSpeed(double speed) {
        return new RunCommand(() -> motor.set(-speed), this).finallyDo(interrupted -> stop());
    }

    /**
     * Command to move the pivot manually at the given speed while a button is held
     *
     * @param speed signed value of speed to run the pivot motor at
     * @return command to run the motor at the given speed
     */
    public Command runPivotAtSpeed(double speed) {
        return new RunCommand(() -> motor.set(speed), this).finallyDo(interrupted -> stop());
    }

    /**
     * return a command that sets the position to the given offset from the ALL_WAY_IN value
     *
     * @param offset difference from the ALL_WAY_IN position to move
     * @return command to set the PID setpoint to the given offset from the base position
     */
    public Command setTargetPositionOffsetCommand(double offset, boolean hold) {
        return setTargetPositionCommand(() -> offset + ALL_IN_POSITION, hold);
    }

    public Command setTargetPositionOffsetCommand(double offset) {
        return setTargetPositionOffsetCommand(offset, false);
    }

    /**
     * set the target on the SparkMax Closed Loop PID to the given position
     *
     * @param position position to move to
     * @return command to set the PID setpoint to the given position
     */
    public Command setTargetPositionCommand(DoubleSupplier position, boolean hold) {
        return new FunctionalCommand(
                () -> setPosition(position),
                this::doNothing,
                (interrupted) -> stop(),
                () -> !hold && onTarget(position),
                this
        );
    }

    public Command setTargetPositionCommand(DoubleSupplier position) {
        return setTargetPositionCommand(position, false);
    }

    private void doNothing() {
    }

    private Command doNothingCmd() {
        return new RunCommand(this::doNothing, this);
    }
}
