package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotNew extends SubsystemBase{

    private static final int PIVOT_MOTOR_ID = 20;

    public static final double CORAL_ANGLE_L1_L2_L3 = 42.0;
    public static final double CORAL_ANGLE_L4 = 60.0;
    public static final double ALGAE_ANGLE = 85.0;
    public static final double ALGAE_BARGE = 50.0;

    private final SparkMax motor;
    private final SparkAbsoluteEncoder encoder;
    private final SparkClosedLoopController pidController;
    private final EndEffector endEffector;

    public PivotNew() {
        this(PIVOT_MOTOR_ID);
    }

    public PivotNew(int motorId) {
        motor = new SparkMax(motorId, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
        pidController = motor.getClosedLoopController();
        endEffector = EndEffector.getInstance(); // there's gotta be a better way

        SparkMaxConfig config = new SparkMaxConfig();

        config
            .inverted(true)
            .idleMode(IdleMode.kBrake);

        config.absoluteEncoder
            .setSparkMaxDataPortConfig()
            .positionConversionFactor(360.0)
            .zeroOffset(0.885)
            .inverted(false);

        config.softLimit
            .reverseSoftLimit(42.0)
            .forwardSoftLimit(85.0)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimitEnabled(true);

        config.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
            .pidf(0.1, 0, 0, 0, ClosedLoopSlot.kSlot0) // empty ee
            .pidf(0.1, 0, 0, 0, ClosedLoopSlot.kSlot1) // holding coral
            .pidf(0.1, 0, 0, 0, ClosedLoopSlot.kSlot2); // holding algae            

        config.closedLoop.maxMotion
            .maxAcceleration(1)
            .maxVelocity(1)
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
            .allowedClosedLoopError(1, ClosedLoopSlot.kSlot0)
            .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1)
            .allowedClosedLoopError(1, ClosedLoopSlot.kSlot2);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);        
    }

    /**
     * Method to get the current position
     *
     * @return - current position of the absolute encoder
     */
    public double getPosition() {
        return encoder.getPosition();
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

    public void setPosition(double position) {
        pidController.setReference(position, SparkMax.ControlType.kPosition, getPIDSlot(), 0);
    }

    public void holdPosition(double position) {
        pidController.setReference(position, SparkMax.ControlType.kPosition, getPIDSlot(), 0);
    }

    public void holdPosition() {
        holdPosition(getPosition());
    }

    public ClosedLoopSlot getPIDSlot() {
        if (endEffector.isHoldingAlgae())
            return ClosedLoopSlot.kSlot2;
        else if (endEffector.isHoldingCoral())
            return ClosedLoopSlot.kSlot1;
        else
            return ClosedLoopSlot.kSlot0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot: Angle", getPosition());
        SmartDashboard.putNumber("Pivot: Current", getCurrent());
    }

    public Command runPivotCommand(double speed) {
        return new RunCommand(() -> runMotor(speed), this).finallyDo(interrupted -> runMotor(0));
    }

    public Command runPivotVariableCommand(DoubleSupplier speed) {
        return new RunCommand(() -> runMotor(speed.getAsDouble()), this).finallyDo(interrupted -> runMotor(0));            
    }

}
