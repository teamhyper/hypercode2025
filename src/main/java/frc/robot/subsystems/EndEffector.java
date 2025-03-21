package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class EndEffector extends SubsystemBase {

    private static EndEffector instance;

    private static final int MOTOR_ID = 16;
    private static final int TOF1_ID = 21;
    private static final int TOF2_ID = 22;
    private static final int TOF3_ID = 23;

    private static final double PEAK_FWD_TORQUE_CURRENT = 80;
    private static final double PEAK_REV_TORQUE_CURRENT = 80;
    
    private static final double TOF_SAMPLE_TIME_MS = 40;
    private static final double CORAL_THRESHOLD = 100.0;
    private static final double ALGAE_IN_THRESHOLD = 75.2;

    private static final double CORAL_INTAKE_SPEED = .25;
    private static final double CORAL_EJECTION_SPEED = .5;
    
    private static final double ALGAE_INTAKE_CURRENT = 20.0;
    private static final double ALGAE_EJECTION_CURRENT = 60.0;    
    private static final double HOLD_CURRENT = 10.0;

    private final TalonFX intakeMotor;
    private final TorqueCurrentFOC torqueCurrentFOC;

    private final TimeOfFlight tof_coral_inner;
    private final TimeOfFlight tof_coral_outer;
    private final TimeOfFlight tof_algae;

    private final Debouncer debouncer;

    public EndEffector() {        

        intakeMotor = new TalonFX(MOTOR_ID, "rio");
        torqueCurrentFOC = new TorqueCurrentFOC(0);

        tof_coral_inner = new TimeOfFlight(TOF1_ID);
        tof_coral_outer = new TimeOfFlight(TOF2_ID);
        tof_algae = new TimeOfFlight(TOF3_ID);

        debouncer = new Debouncer(0.25, DebounceType.kBoth);

        configMotor();
        configSensors();
    }

    public static EndEffector getInstance() {
        if (instance == null) {
            instance = new EndEffector();
        }
        return instance;
    }

    /**
     * Configures the TalonFX settings.
     */
    private void configMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.TorqueCurrent.PeakForwardTorqueCurrent = PEAK_FWD_TORQUE_CURRENT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -PEAK_REV_TORQUE_CURRENT;
        
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
        return sensor.getRange() < range;
    }

    private boolean isDetectingRange(TimeOfFlight sensor, double range1, double range2){
        return sensor.getRange() >= range1 && sensor.getRange() <= range2;
    }

    /**
     * Returns true if the intake is holding algae.
     */
    public boolean isHoldingAlgae() {
        return debouncer.calculate(isDetectingRange(tof_algae, 60, 100));
    }

    /**
     * Returns true if the intake is holding algae.
     */
    public boolean isHoldingCoral() {
        return isDetecting(tof_coral_outer, CORAL_THRESHOLD);
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
        intakeMotor.setControl(torqueCurrentFOC.withOutput(current));
    }

    @Override
    public void periodic() {

        // Motor Metrics
        SmartDashboard.putNumber("EndEffector: Current", intakeMotor.getTorqueCurrent().getValueAsDouble());

        // Coral Detection Metrics
        SmartDashboard.putNumber("EndEffector: Coral Inner Distance", tof_coral_inner.getRange());
        SmartDashboard.putNumber("EndEffector: Coral Outer Distance", tof_coral_outer.getRange());
        SmartDashboard.putBoolean("EndEffector: Coral Inner", isDetecting(tof_coral_inner, CORAL_THRESHOLD));
        SmartDashboard.putBoolean("EndEffector: Coral Outer", isDetecting(tof_coral_outer, CORAL_THRESHOLD));
        SmartDashboard.putBoolean("EndEffector: isHoldingCoral", isHoldingCoral());

        // Coral & Algae Detection Metrics
        SmartDashboard.putNumber("EndEffector: Algae Distance", tof_algae.getRange());
        SmartDashboard.putBoolean("EndEffector: isHoldingAlgae", isHoldingAlgae());
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
        return new RunCommand(() -> runIntake(CORAL_INTAKE_SPEED), this)
                .until(() -> isDetecting(tof_coral_outer, CORAL_THRESHOLD)) // Stop when outer detection is true
                .finallyDo(interrupted -> runIntake(0));
    }

    /**
     * Command to run the intake until `tof_coral_outer` detects coral then stops.
     */
    public Command ejectCoralCommand() {
        return new RunCommand(() -> runIntake(CORAL_EJECTION_SPEED), this)
                .until(() -> !isHoldingCoral()).withTimeout(1.0) // Stop when outer detection is true
                .finallyDo(interrupted -> runIntake(0));
    }

    /**
     * Command to run the intake until current limit is hit then stops.
     */
    public Command intakeAlgaeCommand() {
        return new RunCommand(() -> runIntakeWithTorqueCurrentFOC(ALGAE_INTAKE_CURRENT), this)
            .until(() -> isHoldingAlgae())
                .finallyDo(interrupted -> {
                    runIntakeWithTorqueCurrentFOC(0);
                });
    }

    /**
     * Command to run the intake at holding current.
     */
    public Command holdAlgaeCommand() {
        return new RunCommand(() -> runIntakeWithTorqueCurrentFOC(HOLD_CURRENT), this)
            .finallyDo(interrupted -> {
                runIntakeWithTorqueCurrentFOC(0);
            });
    }

    /**
     * Command to run the intake until algae is ejected then stops.
     */
    public Command ejectAlgaeCommand() {
        return new RunCommand(() -> runIntakeWithTorqueCurrentFOC(-ALGAE_EJECTION_CURRENT), this)
                .until(() -> !isHoldingAlgae())
                .andThen(() -> runIntake(0), this);
    }

    public Command scoreGamePieceCommand() {
        return new InstantCommand(() -> {
            if (isHoldingAlgae()) {
                ejectAlgaeCommand().schedule();
            } else if (isHoldingCoral()) {
                ejectCoralCommand().schedule();
            }
        }, this);
    }

    // ========================= TRIGGERS ======================================

    /**
     * Trigger to run the intake when coral is detected.
     */
    public Trigger getCoralInnerDetectionTrigger() {
        return new Trigger(() -> isDetecting(tof_coral_inner, CORAL_THRESHOLD));
    }

}
