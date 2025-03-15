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
    public static final double ALL_IN_POSITION = 211.5, ALL_OUT_POSITION = 297.0;
    public static final double ALL_IN_POSITION_OFFSET = 0, ALL_OUT_POSITION_OFFSET = ALL_OUT_POSITION - ALL_IN_POSITION, CLEAR_RAMP_OFFSET = 55.0;
    public static final double COLLECT_CORAL_POSITION_OFFSET = 40.0, SCORE_CORAL_L1_POSITION_OFFSET = 0.0, SCORE_CORAL_L2L3_POSITION_OFFSET = 30.0, SCORE_CORAL_L4_POSITION_OFFSET = 48.0;
    public static final double COLLECT_ALGAE_POSITION_OFFSET = ALL_OUT_POSITION_OFFSET, CARRY_ALGAE_POSITION_OFFSET = 60.0, SCORE_ALGAE_POSITION_OFFSET = 60.0;

    // PID coefficients
    // from REV example p = 0.1, i = 1e-4, d = 1
    private static final double P = 0.015, I = 0.0, D = 0.01;
    private static final double P2 = 0.002, I2 = 0.0, D2 = 0.01;
    private static final double aFF = 0.015, MIN_OUTPUT = -0.125, MAX_OUTPUT = 0.25;
    // REV soft limit parameters
    private static final boolean FORWARD_SOFT_LIMIT_ENABLED = true, REVERSE_SOFT_LIMIT_ENABLED = true;    // Max Motion parameters
    private static final int PIVOT_MOTOR_ID = 20;
    private static final double MAX_VELOCITY = 3000.0, MAX_ACCELERATION = MAX_VELOCITY * 100000, ALLOWED_ERROR = 3;
    private static final double G = 1.51, V = 0.78, A = 0.07, S = 0.0;
    private final SparkMax motor;
    private final SparkAbsoluteEncoder encoder;
    private final SparkClosedLoopController pidController;
    private final ArmFeedforward pivotFeedforward = new ArmFeedforward(S, G, V, A);
    private final ClosedLoopSlot pidSlotMain = ClosedLoopSlot.kSlot0;
    private final ClosedLoopSlot pidSlotExtended = ClosedLoopSlot.kSlot1;
    private double target;
    private boolean isUsingSlot1PID = false;
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
                .pid(P2, I2, D2, pidSlotExtended)
                .outputRange(MIN_OUTPUT, MAX_OUTPUT);

        // TODO: tune maxMotion values
        config.closedLoop.maxMotion
                .maxVelocity(MAX_VELOCITY, pidSlotMain)
                .maxAcceleration(MAX_ACCELERATION, pidSlotMain)
                .allowedClosedLoopError(ALLOWED_ERROR, pidSlotMain);

        config.softLimit
                .forwardSoftLimitEnabled(FORWARD_SOFT_LIMIT_ENABLED)
                .reverseSoftLimitEnabled(REVERSE_SOFT_LIMIT_ENABLED)
                .reverseSoftLimit(ALL_IN_POSITION + COLLECT_CORAL_POSITION_OFFSET - ALLOWED_ERROR)
                .forwardSoftLimit(ALL_OUT_POSITION);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController = motor.getClosedLoopController();
        encoder = motor.getAbsoluteEncoder();

        // set our initial position to the current value of the absolute encoder
        setPosition(encoder::getPosition);

        // place the subsystem on SmartDashboard
        SmartDashboard.putData(this);
        setDefaultCommand(setPositionAndHoldCommand());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("On Target", this::onTarget, null);
        builder.addDoubleProperty("Target", this::getTarget, null);
        builder.addDoubleProperty("EncoderPosition (Actual)", this::getPosition, null);
        builder.addDoubleProperty("EncoderPosition (Offset)", () -> getPosition() - ALL_IN_POSITION, null);
        builder.addBooleanProperty("CanMove In", () -> getPosition() > ALL_IN_POSITION + COLLECT_CORAL_POSITION_OFFSET, null);
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
     * Method to set the pivot position using Max Motion Profiling
     *
     * @param position target value for the PID
     */
    public void setPosition(DoubleSupplier position) {
        encoder.getPosition();
        this.target = position.getAsDouble();

        System.out.printf("Pivot.setPosition(): Position: %.2f%n", this.target);

        pidController.setReference(this.target, SparkMax.ControlType.kPosition, pidSlotMain, aFF);
    }

    /**
     * decide which pid slot to use based on the current position
     */
    private void pidSlotBasedOnPositionTarget() {
        double position = encoder.getPosition();
        boolean shouldUseSlot1PID = position >= ALL_OUT_POSITION + COLLECT_ALGAE_POSITION_OFFSET;

        if (shouldUseSlot1PID && !isUsingSlot1PID) {
            isUsingSlot1PID = true;
            pidController.setReference(this.target, SparkMax.ControlType.kPosition, pidSlotExtended);
        }

        if (!shouldUseSlot1PID && isUsingSlot1PID) {
            isUsingSlot1PID = false;
            pidController.setReference(this.target, SparkMax.ControlType.kPosition, pidSlotMain, aFF);
        }
    }

    /**
     * sets the PID setpoint to the current position and then runs continuously so the closed loop PID on
     * the SparkMax holds to that position
     *
     * @return command that sets the PID setpoint to the current position and holds
     */
    public Command setPositionAndHoldCommand() {
        return setPositionAndHoldCommand(encoder::getPosition);
    }

    /**
     * @param position the position to set the pivot to
     * @return command that sets the PID setpoint to the given position and holds
     */
    public Command setPositionAndHoldCommand(DoubleSupplier position) {
        return setTargetPositionOffsetCommand(() -> position.getAsDouble() - ALL_IN_POSITION, true);
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
     * set the target on the SparkMax Closed Loop PID to the given offset from the ALL_IN_POSITION value
     *
     * @param offset position to move to
     * @param hold   should the command continue running when onTarget
     * @return command to set the PID setpoint to the given offset from ALL_IN_POSITION
     */
    public Command setTargetPositionOffsetCommand(double offset, boolean hold) {
        DoubleSupplier offsetPosition = () -> offset + ALL_IN_POSITION;
        return new FunctionalCommand(
                () -> setPosition(offsetPosition),
                this::pidSlotBasedOnPositionTarget,
                (interrupted) -> stop(),
                // assume if the PID output is below 1%, it's struggling to reach the setpoint but is close enough
                () -> !hold && (onTarget(offsetPosition) || Math.abs(motor.getAppliedOutput()) < 0.01),
                this
        );
    }

    /**
     * set the target to the given offset and assume not holding
     */
    public Command setTargetPositionOffsetCommand(double offset) {
        return setTargetPositionOffsetCommand(offset, false);
    }

    /**
     * set the target to the given offset and holding
     */
    public Command setTargetPositionOffsetCommand(DoubleSupplier offset, boolean hold) {
        return this.setTargetPositionOffsetCommand(offset.getAsDouble(), hold);
    }




}
