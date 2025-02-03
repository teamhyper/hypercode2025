package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkCommandResult;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class Pivot extends SubsystemBase {
    private final SparkMax pivotMotor;
    private final SparkAbsoluteEncoder absoluteEncoder;
    private final SparkClosedLoopController pidController;
    private final ProfiledPIDController profiledPIDController;
    private final ArmFeedforward armFeedforward;
    private double targetPosition;

    public Pivot(int motorID) {
        // Create SparkMAX object
        pivotMotor = new SparkMax(motorID, MotorType.kBrushless);

        // Set up configuration
        SparkConfiguration config = new SparkConfiguration();
        config.setSmartCurrentLimit(40);
        config.setIdleMode(SparkMax.IdleMode.kBrake);

        // Apply configuration
        SparkCommandResult result = pivotMotor.setConfiguration(config);
        if (!result.isSuccess()) {
            System.err.println("Spark config error: " + result.getMessage());
        }

        // Create absolute encoder and PID controller
        absoluteEncoder = pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        pidController = pivotMotor.getClosedLoopController();

        // Configure encoder
        absoluteEncoder.setPositionConversionFactor(360.0); // Convert to degrees

        // Configure PID controller
        pidController.setFeedbackDevice(absoluteEncoder);
        pidController.setP(0.1);
        pidController.setI(0.0);
        pidController.setD(0.0);
        pidController.setFF(0.0);

        // Configure trapezoidal motion profile
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(100.0, 200.0);
        profiledPIDController = new ProfiledPIDController(0.1, 0.0, 0.0, constraints);
        profiledPIDController.setGoal(absoluteEncoder.getPosition() * 360.0);

        // Configure feedforward for gravity compensation
        armFeedforward = new ArmFeedforward(0.0, 0.2, 0.0); // Tune kS, kG, kV as needed

        // Set default target position
        targetPosition = absoluteEncoder.getPosition() * 360.0;
    }

    public void setTargetPosition(double position) {
        targetPosition = position;
        profiledPIDController.setGoal(targetPosition);
    }

    public Command pivotToPosition(double position) {
        return new InstantCommand(() -> setTargetPosition(position), this);
    }

    public Command holdPosition() {
        return new RunCommand(() -> {
            double currentAngle = absoluteEncoder.getPosition() * 360.0;
            double output = profiledPIDController.calculate(currentAngle);
            double feedforward = armFeedforward.calculate(Math.toRadians(currentAngle), 0);

            // Combine motion profile output + feedforward
            pidController.setReference(output + feedforward, SparkClosedLoopController.ControlType.kPosition);
        }, this);
    }
}
