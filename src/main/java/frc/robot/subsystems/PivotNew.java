package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class PivotNew extends SubsystemBase{

    private static final int PIVOT_MOTOR_ID = 20;

    public static final double ANGLE_HARDSTOP = 41.0;
    public static final double ANGLE_SAFE_MOVE = 50.0;
    public static final double ANGLE_CORAL_L4 = 60.0;
    public static final double ANGLE_ALGAE_COLLECT = 80.0;
    public static final double ANGLE_BARGE = 41.0;
    public static final double TOLERENCE = 1.0;

    private final SparkMax motor;
    private final SparkAbsoluteEncoder encoder;
    private final SparkClosedLoopController pidController;
    private final ArmFeedforward feedforward;
    private final SysIdRoutine sysIdRoutine;

    private final EndEffector endEffector;

    private double target = 0.0;

    public PivotNew() {
        this(PIVOT_MOTOR_ID);
    }

    public PivotNew(int motorId) {
        motor = new SparkMax(motorId, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
        pidController = motor.getClosedLoopController();
        endEffector = EndEffector.getInstance();

        SparkMaxConfig config = new SparkMaxConfig();

        feedforward = new ArmFeedforward(
            0.0, 
            .75, 
            0.0, 
            0.0
            );

        config
            .inverted(true)
            .idleMode(IdleMode.kBrake);

        config.absoluteEncoder
            .setSparkMaxDataPortConfig()
            .positionConversionFactor(360.0)
            .velocityConversionFactor(360.0)
            .zeroOffset(0.885)
            .inverted(false);

        config.softLimit
            .reverseSoftLimit(41.0)
            .forwardSoftLimit(85.0)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimitEnabled(true);

        config.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
            .pidf(0.0175, 0, 0.05, 0, ClosedLoopSlot.kSlot0) // EMPTY
            .pidf(0.0175, 0, 0.05, 0, ClosedLoopSlot.kSlot1) // HOLDING CORAL
            .pidf(0.0175, 0, 0.05, 0, ClosedLoopSlot.kSlot2) // HOLDING ALGAE
            .pidf(0.025, 0, 0.05, 0, ClosedLoopSlot.kSlot3) // START POSITION PID
            .outputRange(-0.2, 0.2, ClosedLoopSlot.kSlot0)
            .outputRange(-0.2, 0.2, ClosedLoopSlot.kSlot1)
            .outputRange(-0.2, 0.2, ClosedLoopSlot.kSlot2)
            .outputRange(-0.2, 0.2, ClosedLoopSlot.kSlot3);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(), 
            new SysIdRoutine.Mechanism(
                // Drive callback: apply the commanded voltage to the arm motor
                volts -> motor.setVoltage(volts),
                // Log callback: record applied voltage, position, and velocity
                log -> {
                    double appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
                    double angleRadians = Units.degreesToRadians(encoder.getPosition());      // arm angle (degrees)
                    double angularVelocity = Units.degreesToRadians(encoder.getVelocity() / 60.0);   // arm angular velocity (rad/s)
                    log.motor("PivotArmMotor")                                // name the motor for logging
                       .voltage(Voltage.ofBaseUnits(appliedVolts, Volts))
                       .angularPosition(Radians.of(angleRadians))
                       .angularVelocity(RadiansPerSecond.of(angularVelocity));
                },
                this,               // subsystem as requirement
                "PivotArm"          // mechanism name in the log (optional)
            )
        );
    }

    public double getAngle() {
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

    public void setAngle(double position) {
        pidController.setReference(
            position, 
            SparkMax.ControlType.kPosition, 
            getPIDSlot(), 
            feedforward.calculate(Units.degreesToRadians(position), Math.PI), 
            ArbFFUnits.kVoltage);
    }

    public ClosedLoopSlot getPIDSlot() {
        if (getAngle() < 30.0)
            return ClosedLoopSlot.kSlot3;
        else if (endEffector.isHoldingAlgae())
            return ClosedLoopSlot.kSlot2;
        else if (endEffector.isHoldingCoral())
            return ClosedLoopSlot.kSlot1;
        else
            return ClosedLoopSlot.kSlot0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot: Angle", getAngle());
        SmartDashboard.putNumber("Pivot: Current", getCurrent());
    }

    public Command runPivotCommand(double speed) {
        return new RunCommand(() -> runMotor(speed), this)
        .finallyDo(interrupted -> runMotor(0)).andThen(holdPivotAngleCommand()); // TEST
    }

    public Command runPivotVariableCommand(DoubleSupplier speed) {
        return new RunCommand(() -> runMotor(speed.getAsDouble()), this)
        .finallyDo(interrupted -> holdPivotAngleCommand(getAngle())); // TEST        
    }

    public Command jogPivotOutCommand() {
        return new FunctionalCommand(
            () -> target = getAngle() + 3.0,
            () -> setAngle(target),
            (interrupted) -> stop(),
            () -> (Math.abs(target - getAngle()) < TOLERENCE),
            this
        );
    }

    public Command jogPivotInCommand() {
        return new FunctionalCommand(
            () -> target = getAngle() - 3.0,
            () -> setAngle(target),
            (interrupted) -> stop(),
            () -> (Math.abs(target - getAngle()) < TOLERENCE),
            this
        );
    }

    public Command setPivotAngleCommand(double angle) {
        return new FunctionalCommand(
            () -> {},
            () -> setAngle(angle),
            (interrupted) -> stop(),
            () -> (Math.abs(angle - getAngle()) < TOLERENCE),
            this
        );
    }

    public Command holdPivotAngleCommand(double position) {
        return new FunctionalCommand(
            () -> {},
            () -> setAngle(position),
            (interrupted) -> stop(),
            () -> false, // NEVER ENDS
            this
        );        
    }

    public Command holdPivotAngleCommand() {
        return holdPivotAngleCommand(getAngle());
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction); 
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

}
