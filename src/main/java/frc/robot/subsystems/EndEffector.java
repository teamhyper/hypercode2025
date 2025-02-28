package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class EndEffector extends SubsystemBase {
    private static final int MOTOR_ID = 16; // Change this ID based on actual CAN ID
    private static final double HOLD_CURRENT = 10.0; // Maximum holding torque
    private static final double CURRENT_THRESHOLD = 30.0; // Adjust based on expected current spike when grabbing
    private static final double TORQUE_SCALING_FACTOR = 0.5; // Scaling factor for adaptive torque
    private static final double TOF_SAMPLE_TIME_MS = 30;
    private static final double CORAL_THRESHOLD = 100;
    private static final double ALGAE_THRESHOLD = 300;

    private final TalonFX intakeMotor;

    private final TimeOfFlight tof_coral_inner;
    private final TimeOfFlight tof_coral_outer;
    private final TimeOfFlight tof_algae;    


    private boolean isCoralInnerDetected;
    private boolean isCoralOuterDetected;
    private boolean isAlgaeDetected;
    private boolean isHolding;
    private double motorStatorCurrent;

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
        tof_coral_inner.setRangingMode(TimeOfFlight.RangingMode.Short, TOF_SAMPLE_TIME_MS);
        tof_coral_outer.setRangingMode(TimeOfFlight.RangingMode.Short, TOF_SAMPLE_TIME_MS);
        tof_algae.setRangingMode(TimeOfFlight.RangingMode.Short, TOF_SAMPLE_TIME_MS);
    }

    /**
     * Runs the intake at a given percentage output.
     * @param sensor The ToF sensor
     * @param range Range threshold in mm for positive detection
     */
    private boolean isDetecting(TimeOfFlight sensor, double range){
        return sensor.getRange() < range;
    }

    /**
     * Runs the intake at a given percentage output.
     * Positive input intakes algae, intakes coral from rear.
     * Negative input outputs algae, outputs coral from front.
     * @param speed The speed to run the intake (-1.0 to 1.0)
     */
    public void runIntake(double speed) {
        intakeMotor.set(speed);
        isHolding = false;
    }

    /**
     * Runs the intake at a given current output.
     * Positive input intakes algae, intakes coral from rear.
     * Negative input outputs algae, outputs coral from front.
     * @param current The current to run the intake in Amps
     */
    public void runIntakeWithTorqueCurrentFOC(double current) {
        intakeMotor.setControl(new TorqueCurrentFOC(current));
    }

    /**
     * Stops the intake and holds the game piece with adaptive torque control.
     */
    // turn this into a command instead of this
    // public void holdGamePieceAdaptive() {
    //     double current = intakeMotor.getStatorCurrent().getValueAsDouble();
    //     double adaptiveTorque = Math.min(HOLD_CURRENT, current * TORQUE_SCALING_FACTOR); // Scale torque based on resistance
    //     intakeMotor.setControl(new TorqueCurrentFOC(adaptiveTorque));
    //     isHolding = true;
    // } 

    // private boolean isCurrentLimit

    @Override
    public void periodic() {
        motorStatorCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();

        isCoralInnerDetected = isDetecting(tof_coral_inner, CORAL_THRESHOLD);
        isCoralOuterDetected = isDetecting(tof_coral_outer, CORAL_THRESHOLD);
        isAlgaeDetected = isDetecting(tof_algae, ALGAE_THRESHOLD);

        SmartDashboard.putNumber("EndEffector Current", motorStatorCurrent);
        SmartDashboard.putNumber("Adaptive Torque", Math.min(HOLD_CURRENT, motorStatorCurrent * TORQUE_SCALING_FACTOR));

        // Coral & Algae Detection Metrics
        SmartDashboard.putNumber("Coral Inner Distance", tof_coral_inner.getRange());
        SmartDashboard.putNumber("Coral Outer Distance", tof_coral_outer.getRange());
        SmartDashboard.putNumber("Algae Distance", tof_algae.getRange());
        SmartDashboard.putBoolean("Coral Inner", isCoralInnerDetected);
        SmartDashboard.putBoolean("Coral Outer", isCoralOuterDetected);
        SmartDashboard.putBoolean("Algae Present", isAlgaeDetected);
        
        if (isAlgaeDetected && !isHolding && motorStatorCurrent > CURRENT_THRESHOLD) {
            isHolding = true;
            runIntakeWithTorqueCurrentFOC(HOLD_CURRENT);
        }
    }

    // ========================= COMMANDS ======================================

    /**
     * Command to run the intake at a given speed.
     */
    public Command runIntakeCommand(double speed) {
        return new RunCommand(()-> runIntake(speed), this).finallyDo(interrupted->runIntake(0));
    }

    /**
     * Command to run the intake until `tof_coral_outer` detects coral then stops.
     */
    public Command intakeCoralCommand() {
        return new RunCommand(() -> runIntake(0.5), this) // Adjust speed as needed
                .until(() -> isDetecting(tof_coral_outer, CORAL_THRESHOLD)) // Stop when outer detection is true
                .finallyDo(interrupted -> runIntake(0)); // Ensure the motor stops
    }

    public Command intakeAlgaeCommand() {
        return new RunCommand(() -> runIntakeWithTorqueCurrentFOC(10), this)
                .until()

    }

    /**
     * Command to hold the game piece adaptively.
     */
    // public Command holdGamePieceCommand() {
    //     return new InstantCommand(this::holdGamePieceAdaptive, this);
    // }

    /**
     * Continuous command to run the intake at a given speed until interrupted.
     */
    public Command runIntakeContinuousCommand(double speed) {
        return new RunCommand(() -> runIntake(speed), this);
    }

    // public Command autoIntakeAndHoldCommand(double intakeSpeed) {
    //     return new RunCommand(() -> {
    //         double current = intakeMotor.getStatorCurrent().getValueAsDouble();
    //         SmartDashboard.putNumber("EndEffector Current", current);
            
    //         if (current > CURRENT_THRESHOLD) {
    //             holdGamePieceAdaptive(); // Automatically switch to holding mode
    //         } else {
    //             runIntake(intakeSpeed);
    //         }
    //     }, this);
    // }

    // ========================= TRIGGERS ======================================
    public Trigger getCoralInnerDetectionTrigger() {
        return new Trigger(() -> isDetecting(tof_coral_inner, CORAL_THRESHOLD));
    }

    public Trigger getAlgaeDetectionTrigger() {
        return new Trigger(() -> isDetecting(tof_algae, ALGAE_THRESHOLD));
    }
    
}
