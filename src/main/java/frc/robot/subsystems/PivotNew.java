package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotNew extends SubsystemBase{

    private static final int PIVOT_MOTOR_ID = 20;

    private static final double ABS_ENC_OFFSET = 0;

    private final SparkMax motor;
    private final SparkAbsoluteEncoder encoder;

    public PivotNew() {
        this(PIVOT_MOTOR_ID);
    }

    public PivotNew(int motorId) {
        motor = new SparkMax(motorId, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();

        SparkMaxConfig config = new SparkMaxConfig();

        // add current limiting
        // add spark max motion profile

        config
            .inverted(false)
            .idleMode(IdleMode.kBrake);

        config.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(0)
            .forwardSoftLimit(90);

        config.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
            .pidf(1, 0, 0, 0, null);

        config.closedLoop.maxMotion
            .maxAcceleration(0)
            .maxVelocity(motorId)
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        config.absoluteEncoder
            .inverted(true)
            .positionConversionFactor(360);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putData(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        // builder.addBooleanProperty("On Target", this::onTarget, null);
        // builder.addDoubleProperty("Target", this::getTarget, null);
        // builder.addDoubleProperty("EncoderPosition (Actual)", this::getPosition, null);
        // builder.addDoubleProperty("EncoderPosition (Offset)", () -> getPosition() - ALL_IN_POSITION, null);
        // builder.addBooleanProperty("CanMove In", () -> getPosition() > ALL_IN_POSITION + COLLECT_CORAL_POSITION_OFFSET, null);
        // builder.addBooleanProperty("CanMove Out", () -> getPosition() < ALL_OUT_POSITION, null);
        // builder.addDoubleProperty("Speed", motor::get, null);
        // builder.addDoubleProperty("Applied Output", motor::getAppliedOutput, null);
    }

    /**
     * Method to get the current position
     *
     * @return - current position of the absolute encoder
     */
    public double getPosition() {
        return encoder.getPosition() - ABS_ENC_OFFSET;
    }

    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    private void runMotor(double speed) {
        motor.set(speed);        
    }

    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }

}
