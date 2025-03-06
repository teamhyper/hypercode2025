package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
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
    public static final double COLLECT_CORAL_POSITION_OFFSET = 35.0, COLLECT_ALGAE_POSITION_OFFSET = 94.0, SCORE_ALGAE_POSITION_OFFSET = 0.0, SCORE_CORAL_POSITION_OFFSET = 48.0, CARRY_ALGAE_POSITION_OFFSET = 60.0;
    public static final double ALL_IN_POSITION = 211.5, ALL_OUT_POSITION = 306;
    // PID coefficients
    // from REV example p = 0.1, i = 1e-4, d = 1
    private static final double kP = 0.01, kI = 0.0, kD = 0.0001;
    private static final double kFF = 0.0, kMinOutput = -1.0, kMaxOutput = 1.0;
    // Max Motion parameters
    private static final double kMaxVelocity = 1000.0, kMaxAcceleration = 500.0, kAllowedError = 0.001;
    // REV soft limit parameters
    private static final boolean kForwardSoftLimitEnabled = true, kReverseSoftLimitEnabled = true;
    private static final int PIVOT_MOTOR_ID = 20;
    private static final double kS = 0, kG = 0, kV = 0, kA = 0;
    private final SparkMax m_pivot;
    private final SparkAbsoluteEncoder encoder;
    private final SparkClosedLoopController pidController;
    private final ArmFeedforward pivotFeedforward = new ArmFeedforward(kS, kG, kV, kA);
    private double target;


    public Pivot() {
        this(PIVOT_MOTOR_ID);
    }

    public Pivot(int motorId) {
        m_pivot = new SparkMax(motorId, MotorType.kBrushless);
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
                .pid(kP, kI, kD)
                .outputRange(kMinOutput, kMaxOutput);

        // TODO: tune maxMotion values
        config.closedLoop.maxMotion
                .maxVelocity(kMaxVelocity)
                .maxAcceleration(kMaxAcceleration)
                .allowedClosedLoopError(kAllowedError);

        config.softLimit
                .forwardSoftLimitEnabled(kForwardSoftLimitEnabled)
                .reverseSoftLimitEnabled(kReverseSoftLimitEnabled)
                .reverseSoftLimit(ALL_IN_POSITION)
                .forwardSoftLimit(ALL_OUT_POSITION);

        m_pivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController = m_pivot.getClosedLoopController();
        encoder = m_pivot.getAbsoluteEncoder();

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
        builder.addBooleanProperty("CanMove In", () -> getPosition() > ALL_IN_POSITION, null);
        builder.addBooleanProperty("CanMove Out", () -> getPosition() < ALL_OUT_POSITION, null);
        builder.addDoubleProperty("Speed", m_pivot::get, null);
        builder.addDoubleProperty("Applied Output", m_pivot::getAppliedOutput, null);
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
        this.target = position.getAsDouble();

        System.out.printf("Pivot.setPosition(): Position: %.2f%n", this.target);
        pidController.setReference(this.target, SparkMax.ControlType.kMAXMotionPositionControl); // testing if this is why the "go to positions" commands operate so slowly
//        pidController.setReference(this.target, SparkMax.ControlType.kPosition);
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
        m_pivot.stopMotor();
    }

    /**
     *
     * @return the stored target value
     */
    private double getTarget() {
        return this.target;
    }

    /**
     * is the Pivot encoder position at the given target
     * @param target the specific target position to check against
     * @return if the Pivot is at the given target
     */
    public boolean onTarget(DoubleSupplier target) {
        return Math.abs(encoder.getPosition() - target.getAsDouble()) <= kAllowedError;
    }

    /**
     * is the Pivot encoder position at the previously set target
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
        return new RunCommand(() -> m_pivot.set(speed), this).finallyDo(interrupted -> stop());
    }

    /**
     * Command to move pivot manually inward while a button is held
     *
     * @param speed absolute value of speed to run the pivot motor inwards at
     * @return command to run the motor inward at a speed
     */
    public Command runPivotInAtSpeed(double speed) {
        return new RunCommand(() -> m_pivot.set(-speed), this).finallyDo(interrupted -> stop());
    }

    /**
     * Command to move the pivot manually at the given speed while a button is held
     *
     * @param speed signed value of speed to run the pivot motor at
     * @return command to run the motor at the given speed
     */
    public Command runPivotAtSpeed(double speed) {
        return new RunCommand(() -> m_pivot.set(speed), this).finallyDo(interrupted -> stop());
    }

    /**
     * return a command that sets the position to the given offset from the ALL_WAY_IN value
     *
     * @param offset difference from the ALL_WAY_IN position to move
     * @return command to set the PID setpoint to the given offset from the base position
     */
    public Command setTargetPositionOffsetCommand(double offset, boolean hold) {
        return setTargetPositionCommand(() -> {
            double position = offset + ALL_IN_POSITION;
            System.out.printf("Pivot.setPositionOffset(): Offset: %.2f, Calculated: %.2f%n", offset, position);
            return position;
        }, hold);
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
