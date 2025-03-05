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

    private static final double CLIMBER_ENCODER_OFFSET = 10; // TODO: set when calculated
    private static final double PREPARE_CURRENT = 30.0;
    private static final double CLIMB_CURRENT = 80.0;

    private final TalonFX m_climber;
    private final CANcoder e_climber;
    
    public Climber() {
        this(CLIMBER_ID, CLIMBER_ABS_ENC_ID);
    }

    public Climber(int climberID, int climberEncoderID) {
        m_climber = new TalonFX(climberID, "hyperbus");
        e_climber = new CANcoder(climberEncoderID, "hyperbus");

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

    @Override
    public void periodic() {

      SmartDashboard.putNumber("Climber Angle", getClimberAngle());
      SmartDashboard.putNumber("Climber Current Output", m_climber.getTorqueCurrent().getValueAsDouble());
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

    /**
     * Command to stop the climber.
     */
    public Command stopClimberCommand() {
        return new InstantCommand(() -> runClimber(0), this);
    }

    /**
     * Command to move the climber to climbing position.
     */
    public Command rotateClimberOutCommand() {
        return rotateClimberCommand(-PREPARE_CURRENT).until(() -> getClimberAngle() >= 180).andThen(stopClimberCommand());
    }

    /**
     * Command to move the climber to lifting position.
     */
    public Command rotateClimberInCommand() {
        return rotateClimberCommand(CLIMB_CURRENT).until(() -> getClimberAngle() <= 100).andThen(stopClimberCommand());
    }

}