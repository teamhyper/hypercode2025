package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
    private static final int MOTOR_ID = 16; // Change this ID based on actual CAN ID
    private static final double MAX_HOLD_TORQUE = 20.0; // Maximum holding torque
    private static final double CURRENT_THRESHOLD = 30.0; // Adjust based on expected current spike when grabbing
    private static final double TORQUE_SCALING_FACTOR = 0.5; // Scaling factor for adaptive torque

    private final TalonFX intakeMotor;

    private final TimeOfFlight tof_coral_inner;
    private final TimeOfFlight tof_coral_outer;
    private final TimeOfFlight tof_algae;
    private boolean isHolding = false;

    public EndEffector() {
        intakeMotor = new TalonFX(MOTOR_ID, "rio");
        tof_coral_inner = new TimeOfFlight(21);
        tof_coral_outer = new TimeOfFlight(22);
        tof_algae = new TimeOfFlight(23);

        configMotor();
        configSensors();
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

    private void configSensors() {
        tof_coral_inner.setRangingMode(TimeOfFlight.RangingMode.Short, 50);
        tof_coral_outer.setRangingMode(TimeOfFlight.RangingMode.Short, 50);
        tof_algae.setRangingMode(TimeOfFlight.RangingMode.Short, 50);
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
     * Stops the intake and holds the game piece with adaptive torque control.
     */
    public void holdGamePieceAdaptive() {
        double current = intakeMotor.getStatorCurrent().getValueAsDouble();
        double adaptiveTorque = Math.min(MAX_HOLD_TORQUE, current * TORQUE_SCALING_FACTOR); // Scale torque based on resistance
        intakeMotor.setControl(new TorqueCurrentFOC(adaptiveTorque));
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
        double current = intakeMotor.getStatorCurrent().getValueAsDouble(); // Get current draw

        double coral_inner_distance = tof_coral_inner.getRange();
        double coral_outer_distance = tof_coral_outer.getRange();
        double algae_distance = tof_algae.getRange();

        SmartDashboard.putNumber("EndEffector Current", current);
        SmartDashboard.putNumber("Adaptive Torque", Math.min(MAX_HOLD_TORQUE, current * TORQUE_SCALING_FACTOR));

        SmartDashboard.putNumber("Coral Inner Distance", coral_inner_distance);
        SmartDashboard.putNumber("Coral Outer Distance", coral_outer_distance);
        SmartDashboard.putNumber("Algae Distance", algae_distance);
        
        if (!isHolding && current > CURRENT_THRESHOLD) {
            holdGamePieceAdaptive(); // Switch to adaptive torque control
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
     * Command to hold the game piece adaptively.
     */
    public Command holdGamePieceCommand() {
        return new InstantCommand(this::holdGamePieceAdaptive, this);
    }

    /**
     * Continuous command to run the intake at a given speed until interrupted.
     */
    public Command runIntakeContinuousCommand(double speed) {
        return new RunCommand(() -> runIntake(speed), this);
    }

    public Command autoIntakeAndHoldCommand(double intakeSpeed) {
        return new RunCommand(() -> {
            double current = intakeMotor.getStatorCurrent().getValueAsDouble();
            SmartDashboard.putNumber("EndEffector Current", current);
            
            if (current > CURRENT_THRESHOLD) {
                holdGamePieceAdaptive(); // Automatically switch to holding mode
            } else {
                runIntake(intakeSpeed);
            }
        }, this);
    }
    
}
