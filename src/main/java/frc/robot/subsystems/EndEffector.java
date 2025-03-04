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
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class EndEffector extends SubsystemBase {
    private static final int MOTOR_ID = 16;
    private static final int TOF1_ID = 21;
    private static final int TOF2_ID = 22;
    private static final int TOF3_ID = 23;
    private static final double HOLD_CURRENT = 5.0;
    private static final double TOF_SAMPLE_TIME_MS = 30;
    private static final double CORAL_THRESHOLD = 100;
    private static final double ALGAE_OUT_THRESHOLD = 250;
    private static final double ALGAE_IN_THRESHOLD = 100;
    private static final double CORAL_EJECTION_SPEED = .5;
    private static final double ALGAE_INTAKE_CURRENT = 20.0;
    private static final double ALGAE_EJECTION_CURRENT = 40.0;

    private final TalonFX intakeMotor;

    private final TimeOfFlight tof_coral_inner;
    private final TimeOfFlight tof_coral_outer;
    private final TimeOfFlight tof_algae;

    public EndEffector() {        

        intakeMotor = new TalonFX(MOTOR_ID, "rio");
        tof_coral_inner = new TimeOfFlight(TOF1_ID);
        tof_coral_outer = new TimeOfFlight(TOF2_ID);
        tof_algae = new TimeOfFlight(TOF3_ID);

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

    /**
     * Configures subsystem sensor settings.
     */
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
        // TODO: Test if isRangeValid() works as intended
        return sensor.getRange() < range && sensor.isRangeValid();
    }

    /**
     * Returns true if the intake is holding algae.
     */
    private boolean isHoldingAlgae() {
        return isDetecting(tof_algae, ALGAE_OUT_THRESHOLD) && isDetecting(tof_algae, ALGAE_IN_THRESHOLD);
    }

    /**
     * Runs the intake at a given percentage output.
     * Positive input intakes algae, intakes coral from rear.
     * Negative input outputs algae, outputs coral from front.
     * @param speed The speed to run the intake (-1.0 to 1.0)
     */
    public void runIntake(double speed) {
        intakeMotor.set(speed);
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

    @Override
    public void periodic() {

        // Motor Metrics
        SmartDashboard.putNumber("EndEffector Current", intakeMotor.getStatorCurrent().getValueAsDouble());

        // Coral Detection Metrics
        SmartDashboard.putNumber("Coral Inner Distance", tof_coral_inner.getRange());
        SmartDashboard.putNumber("Coral Outer Distance", tof_coral_outer.getRange());
        SmartDashboard.putBoolean("Coral Inner", isDetecting(tof_coral_inner, CORAL_THRESHOLD));
        SmartDashboard.putBoolean("Coral Outer", isDetecting(tof_coral_outer, CORAL_THRESHOLD));

        // Coral & Algae Detection Metrics
        SmartDashboard.putNumber("Algae Distance", tof_algae.getRange());        
        SmartDashboard.putBoolean("Algae Present", isDetecting(tof_algae, ALGAE_OUT_THRESHOLD));
        SmartDashboard.putBoolean("Algae Present Close", isDetecting(tof_algae, ALGAE_IN_THRESHOLD));
        SmartDashboard.putBoolean("isHoldingAlgae", isHoldingAlgae());
    }

    // ========================= COMMANDS ======================================

    /**
     * Command to run the intake at a given speed.
     */
    public Command runIntakeCommand(double speed) {
        return new RunCommand(()-> runIntake(speed), this).finallyDo(interrupted->runIntake(0));
    }

    /**
     * Command to stop the intake.
     */
    public Command stopIntakeCommand() {
        return new InstantCommand(()-> runIntake(0), this);
    }

    /**
     * Command to run the intake until `tof_coral_outer` detects coral then stops.
     */
    public Command intakeCoralCommand() {
        return new RunCommand(() -> runIntake(0.5), this)
                .until(() -> isDetecting(tof_coral_outer, CORAL_THRESHOLD)) // Stop when outer detection is true
                .finallyDo(interrupted -> runIntake(0));
    }

    /**
     * Command to run the intake until `tof_coral_outer` detects coral then stops.
     */
    public Command ejectCoralCommand() {
        return new RunCommand(() -> runIntake(CORAL_EJECTION_SPEED), this)
                .until(() -> !isDetecting(tof_coral_outer, CORAL_THRESHOLD)) // Stop when outer detection is true
                .finallyDo(interrupted -> runIntake(0));
    }

    /**
     * Command to run the intake until algae is detected close then holds.
     */
    public Command intakeAlgaeCommand() {
        return new RunCommand(() -> runIntakeWithTorqueCurrentFOC(ALGAE_INTAKE_CURRENT), this)
                .until(() -> isHoldingAlgae()).andThen(() -> runIntakeWithTorqueCurrentFOC(HOLD_CURRENT), this);
    }

    /**
     * Command to run the intake until algae is ejected then stops.
     */
    public Command ejectAlgaeCommand() {
        return new RunCommand(() -> runIntakeWithTorqueCurrentFOC(-ALGAE_EJECTION_CURRENT), this)
                .until(() -> !isHoldingAlgae()).andThen(() -> runIntake(0), this);
    }

    /**
     * Continuous command to run the intake at a given speed until interrupted.
     */
    public Command runIntakeContinuousCommand(double speed) {
        return new RunCommand(() -> runIntake(speed), this);
    }

    // ========================= TRIGGERS ======================================

    /**
     * Trigger to run the intake when coral is detected.
     */
    public Trigger getCoralInnerDetectionTrigger() {
        return new Trigger(() -> isDetecting(tof_coral_inner, CORAL_THRESHOLD));
    }

    /**
     * Trigger to run the intake when algae is detected.
     */
    public Trigger getAlgaeDetectionTrigger() {
        return new Trigger(() -> (isDetecting(tof_algae, ALGAE_OUT_THRESHOLD) && !isDetecting(tof_algae, ALGAE_IN_THRESHOLD)));
    }
    
}
