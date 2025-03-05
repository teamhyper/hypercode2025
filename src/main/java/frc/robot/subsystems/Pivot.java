package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    // TODO: get positions
    // SCORE_CORAL_POSITION_OFFSET is a guesstimate of the midpoint (45deg) between the ALL_IN and ALL_OUT positions
    public static final double COLLECT_CORAL_POSITION_OFFSET = 0.0, COLLECT_ALGAE_POSITION_OFFSET = 0.0, SCORE_ALGAE_POSITION_OFFSET = 0.0, SCORE_CORAL_POSITION_OFFSET = 48.0;
    public static final double ALL_IN_POSITION = 212.5, ALL_OUT_POSITION = 308.5;
    // PID coefficients
    // from REV example p = 0.1, i = 1e-4, d = 1
    private static final double kP = 0.0075, kI = 0.0, kD = 0.0;
    private static final double kFF = 0.0, kMinOutput = -1.0, kMaxOutput = 1.0;
    // Max Motion parameters
    private static final double kMaxVelocity = 1000.0, kMaxAcceleration = 500.0, kAllowedError = 0.005;
    // REV soft limit parameters
    private static final boolean kForwardSoftLimitEnabled = true, kReverseSoftLimitEnabled = true;
    private static final int PIVOT_MOTOR_ID = 20;
    private static final double kS = 0, kG = 0, kV = 0, kA = 0;

    private final SparkMax m_pivot;
    private final SparkAbsoluteEncoder encoder;
    private final SparkClosedLoopController pidController;
    private final ArmFeedforward pivotFeedforward = new ArmFeedforward(kS, kG, kV, kA);


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
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(kP, kI, kD, kFF)
                .minOutput(kMinOutput)
                .maxOutput(kMaxOutput);

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

        setDefaultCommand(setPositionAndHoldCommand());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot: EncoderPosition (Actual)", getPosition());
        SmartDashboard.putNumber("Pivot: EncoderPosition (Offset)", getPosition() - ALL_IN_POSITION);
        SmartDashboard.putBoolean("Pivot: CanMove In", getPosition() > ALL_IN_POSITION);
        SmartDashboard.putBoolean("Pivot: CanMove Out", getPosition() < ALL_OUT_POSITION);
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
     * @param position - target value for the PID
     */
    public void setPosition(double position) {
        pidController.setReference(position, SparkMax.ControlType.kMAXMotionPositionControl);
    }

    /**
     * sets the PID setpoint to the current position and then runs continuously so the closed loop PID on
     * the SparkMax holds to that position
     *
     * @return command that sets the PID setpoint to the current position and holds
     */
    public Command setPositionAndHoldCommand() {
        return setTargetPositionCommand().andThen(
                new RunCommand(() -> {
                }, this));
    }

    /**
     * set the target position for the PID based on the given offset from the base position
     *
     * @param offset - distance from the base position
     */
    public void setPositionOffset(double offset) {
        setPosition(offset + ALL_IN_POSITION);
    }

    /**
     * stop the motor (speed 0)
     */
    public void stop() {
        m_pivot.stopMotor();
    }

    /**
     * Command to move pivot manually outward while a button is held
     *
     * @param speed - speed to run the pivot motor at
     * @return command to run the motor at positive speed
     */
    public Command runPivotOut(double speed) {
        return new RunCommand(() -> m_pivot.set(speed), this).finallyDo(interrupted -> stop());
    }

    /**
     * Command to move pivot manually inward while a button is held
     *
     * @param speed - speed to run the pivot motor at
     * @return command to run the motor at negative speed
     */
    public Command runPivotIn(double speed) {
        return new RunCommand(() -> m_pivot.set(-speed), this).finallyDo(interrupted -> stop());
    }

    /**
     * return a command that sets the position to the given offset from the ALL_WAY_IN value
     *
     * @param offset - difference from the ALL_WAY_IN position to move
     * @return command to set the PID setpoint to the given offset from the base position
     */
    public Command setTargetPositionOffsetCommand(double offset) {
        return new InstantCommand(() -> setPositionOffset(offset));
    }

    /**
     * set the target on the SparkMax Closed Loop PID to the given position
     *
     * @param position - position to move to
     * @return command to set the PID setpoint to the given position
     */
    public Command setTargetPositionCommand(double position) {
        return new InstantCommand(() -> setPosition(position));
    }

    /**
     * set the target on the SparkMax Closed Loop PID to the current position of the absolute encoder
     *
     * @return command to set the PID setpoint to the current position
     */
    public Command setTargetPositionCommand() {
        return setTargetPositionCommand(m_pivot.getAbsoluteEncoder().getPosition());
    }
}
