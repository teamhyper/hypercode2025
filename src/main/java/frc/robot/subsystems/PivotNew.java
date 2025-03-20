package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
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

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotNew extends SubsystemBase{

    private static final int PIVOT_MOTOR_ID = 20;

    private static final double ABS_ENC_OFFSET = 8.085;

    private final SparkMax motor;
    private final SparkAbsoluteEncoder encoder;
    private final SparkClosedLoopController pidController;

    public PivotNew() {
        this(PIVOT_MOTOR_ID);
    }

    public PivotNew(int motorId) {
        motor = new SparkMax(motorId, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
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
            .pidf(0.1, 0, 0, 0, ClosedLoopSlot.kSlot0)
            .pidf(0.1, 0, 0, 0, ClosedLoopSlot.kSlot1)
            .pidf(0.1, 0, 0, 0, ClosedLoopSlot.kSlot2);

        // config.closedLoop.maxMotion
        //     .maxAcceleration(0)
        //     .maxVelocity(motorId)
        //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidController = motor.getClosedLoopController();
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
        pidController.setReference(position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
    }

    public ClosedLoopSlot getSlot() {
        // should return slot based on game piece being held
        return ClosedLoopSlot.kSlot0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Angle", getPosition());
    }

    public Command runPivotCommand(double speed) {
        return new RunCommand(() -> runMotor(speed), this).finallyDo(interrupted -> runMotor(0));
    }

    public Command runPivotVariableCommand(DoubleSupplier speed) {
        return new RunCommand(() -> runMotor(speed.getAsDouble()), this).finallyDo(interrupted -> runMotor(0));            
    }

}
