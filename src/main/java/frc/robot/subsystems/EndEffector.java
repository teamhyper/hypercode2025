package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
    private static final int MOTOR_ID = 16; // Change this ID based on actual CAN ID
    private static final double HOLD_TORQUE = 20.0; // Adjust based on actual holding torque required
    private static final double CURRENT_THRESHOLD = 30.0; // Adjust based on expected current spike when grabbing
    
    private final TalonFX intakeMotor;
    private boolean isHolding = false;

    public EndEffector() {
        intakeMotor = new TalonFX(MOTOR_ID);
        configMotor();
        // setDefaultCommand(new RunCommand(this::holdGamePiece, this));
    }

    /**
     * Configures the TalonFX settings.
     */
    private void configMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.1; // Example PID values, adjust as needed
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.TorqueCurrent.PeakForwardTorqueCurrent = 30;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -30;  // Example peak torque current limit
        
        intakeMotor.getConfigurator().apply(config);
    }

    /**
     * Runs the intake at a given percentage output.
     * @param speed The speed to run the intake (-1.0 to 1.0)
     */
    public void runIntake(double speed) {
        intakeMotor.set(speed);
        isHolding = false;
    }

    /**
     * Stops the intake and holds the game piece with a constant torque.
     */
    public void holdGamePiece() {
        intakeMotor.setControl(new TorqueCurrentFOC(HOLD_TORQUE));
        isHolding = true;
    }

    /**
     * Stops the intake motor completely.
     */
    public void stopIntake() {
        intakeMotor.set(0);
        isHolding = false;
    }

    @Override
    public void periodic() {
        double current = intakeMotor.getStatorCurrent().getValueAsDouble(); // Fixed type mismatch
        SmartDashboard.putNumber("EndEffector Current", current); // Publish to Shuffleboard
        if (!isHolding && current > CURRENT_THRESHOLD) {
            holdGamePiece();
        }
    }

    /**
     * Command to run the intake at a given speed.
     */
    public Command runIntakeCommand(double speed) {
        return new InstantCommand(() -> runIntake(speed), this);
    }

    /**
     * Command to stop the intake.
     */
    public Command stopIntakeCommand() {
        return new InstantCommand(this::stopIntake, this);
    }

    /**
     * Command to hold the game piece.
     */
    public Command holdGamePieceCommand() {
        return new InstantCommand(this::holdGamePiece, this);
    }

    public void holdGamePieceAdaptive() {
        double current = intakeMotor.getStatorCurrent().getValueAsDouble();
        double adjustedTorque = Math.min(10.0, current * 0.2); // Scale torque based on current
        intakeMotor.setControl(new TorqueCurrentFOC(adjustedTorque));
        isHolding = true;
    }
    

    /**
     * Continuous command to run the intake at a given speed until interrupted.
     */
    public Command runIntakeContinuousCommand(double speed) {
        return new RunCommand(() -> runIntake(speed), this);
    }
}
