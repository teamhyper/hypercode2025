package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    
    private static final int CLIMBER_ID = 19;
    private static final int CLIMBER_ABS_ENC_ID = 24;
    private static final int CLIMBER_RATCHET_SERVO_PWM_ID = 2; //TODO: pick a port
    private static final int RAMP_LINEAR_SERVO_PWM_ID = 3; //TODO: pick a port

    private static final double RATCHET_LOCK_POSITION = 1.0; // TODO
    private static final double RATCHET_UNLOCK_POSITION  = 0.0;

    private static final double CLIMBER_ENCODER_OFFSET = 10; // TODO: set when calculated

    private final TalonFX m_climber;
    private final CANcoder e_climber;
    private final Servo s_ratchet;
    private final Servo s_ramp;

    private double ratchetLastPosition;

    public Climber() {
        this(CLIMBER_ID, CLIMBER_ABS_ENC_ID);
    }

    public Climber(int climberID, int climberEncoderID) {
        m_climber = new TalonFX(climberID);
        e_climber = new CANcoder(climberEncoderID, "hyperbus");
        s_ratchet = new Servo(CLIMBER_RATCHET_SERVO_PWM_ID);
        s_ramp = new Servo(RAMP_LINEAR_SERVO_PWM_ID);

        configMotors();
        configSensors();
    }

    /**
     * Configures the TalonFX motor settings.
     */
    private void configMotors() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        m_climber.getConfigurator().apply(config);

    }

    /**
     * Configures the TalonFX encoder settings.
     */
    private void configSensors() {
        
        CANcoderConfiguration config = new CANcoderConfiguration();
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();

        // magnetConfig
        
    }

    /**
     * Returns the climber absolute position in degrees.
     */
    private double getClimberAngle() {
        return e_climber.getPosition().getValueAsDouble() - CLIMBER_ENCODER_OFFSET; // TODO: fix
    }

    /**
     * Returns boolean of last commanded ratchet position.
     */
    private boolean getRatchetLocked() {
        return s_ratchet.get() == RATCHET_LOCK_POSITION;
    }

    /**
     * Runs the climber at a given current output.
     * @param current Current output in Amps
     */
    public void runClimber(double current) {
        m_climber.setControl(new TorqueCurrentFOC(current));
    }

    /**
     * Runs the climber at a given current output.
     * @param output Speed output in duty cycle
     */
    public void runClimberVariable(double output) {
        m_climber.setControl(new DutyCycleOut(output));
    }

    /**
     * Set ratchet servo position.
     * @param position Servo position between 0.0 to 1.0
     */
    public void setRatchet(double position) {
        s_ratchet.set(position);
        ratchetLastPosition = position;
    }

    @Override
    public void periodic() {

      SmartDashboard.putNumber("Climber Angle", getClimberAngle());
      SmartDashboard.putNumber("Climber Current Output", m_climber.getTorqueCurrent().getValueAsDouble());

      SmartDashboard.putBoolean("Ratchet Locked", getRatchetLocked());
    }

    /**
     * Command to rotate the climber out.
     * @param current The current to run the climber in Amps
     */
    public Command rotateClimberCommand(double current) {
        return new RunCommand(() -> runClimber(current), this).finallyDo(interrupted -> runClimber(0));
    }

    /**
     * Command to rotate the climber variable.
     * @param output The current to run the climber in duty cycle
     */
    public Command rotateClimberVariableCommad(DoubleSupplier output) {
        return new RunCommand(() -> runClimberVariable(output.getAsDouble()), this).finallyDo(interrupted -> runClimberVariable(0));
    }

    // may need to move rachets to their own subsystem
    /**
     * Command to lock the ratchet.
     */
    public Command lockRatchetCommand() {
        return new InstantCommand(() -> setRatchet(RATCHET_LOCK_POSITION), this);
    }

    public Command unlockRatchetCommand() {
        return new InstantCommand(() -> setRatchet(RATCHET_UNLOCK_POSITION), this);
    }

    

}