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
    // PID coefficients
    // from REV example p = 0.1, i = 1e-4, d = 1
    private static final double kP = 0.0075, kI = 0.0, kD = 0.0;
    private static final double kFF = 0.0, kMinOutput = -1.0, kMaxOutput = 1.0;

    // Max Motion parameters
    private static final double kMaxVelocity = 1000.0, kMaxAcceleration = 500.0, kAllowedError = 0.005;

    // REV soft limit parameters
    private static final boolean kForwardSoftLimitEnabled = true, kReverseSoftLimitEnabled = true;
    private static final double kReverseSoftLimit = 212.5, kForwardSoftLimit = 308.5;

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

        config.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(kP, kI, kD, kFF)
                .minOutput(kMinOutput)
                .maxOutput(kMaxOutput);


        config.closedLoop.maxMotion
                .maxVelocity(kMaxVelocity)
                .maxAcceleration(kMaxAcceleration)
                .allowedClosedLoopError(kAllowedError);

        config.softLimit
                .forwardSoftLimitEnabled(kForwardSoftLimitEnabled)
                .reverseSoftLimitEnabled(kReverseSoftLimitEnabled)
                .reverseSoftLimit(kReverseSoftLimit)
                .forwardSoftLimit(kForwardSoftLimit);

        m_pivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController = m_pivot.getClosedLoopController();
        encoder = m_pivot.getAbsoluteEncoder();

        setDefaultCommand(this.setTargetPositionCommand().andThen(
                new RunCommand(() -> {
                }, this)));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("EncoderPosition", this.getPosition());
    }

    // Method to get the current position
    public double getPosition() {
        return encoder.getPosition();
    }

    // Method to set the pivot position using Max Motion Profiling
    public void setPosition(double position) {
        pidController.setReference(position, SparkMax.ControlType.kMAXMotionPositionControl);
    }

    // Method to stop the pivot
    public void stop() {
        m_pivot.stopMotor();
    }

    // Command to move pivot manually while a button is held
    public Command runPivotOut(double speed) {
        return new RunCommand(() -> m_pivot.set(speed), this).finallyDo(interrupted -> stop());
    }

    // Command to move pivot manually while a button is held
    public Command runPivotIn(double speed) {
        return new RunCommand(() -> m_pivot.set(-speed), this).finallyDo(interrupted -> stop());
    }

    public Command setTargetPositionCommand(double position) {
        return new InstantCommand(() -> setPosition(position));
    }

    public Command setTargetPositionCommand() {
        return this.setTargetPositionCommand(m_pivot.getAbsoluteEncoder().getPosition());
    }
}
