package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    
    private static final int CLIMBER_ID = 19;
    private static final int CLIMBER_ABS_ENC_ID = 24;

    private static final double PREPARE_CURRENT = 30.0;
    private static final double CLIMB_CURRENT = 120.0;

    private final TalonFX motor;
    private final CANcoder encoder;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final DutyCycleOut dutyCycleOut;
    
    public Climber() {
        this(CLIMBER_ID, CLIMBER_ABS_ENC_ID);
    }

    public Climber(int climberID, int climberEncoderID) {
        motor = new TalonFX(climberID, "hyperbus");
        encoder = new CANcoder(climberEncoderID, "hyperbus");
        torqueCurrentFOC = new TorqueCurrentFOC(0);
        dutyCycleOut = new DutyCycleOut(0);

        configMotors();
        configSensors();
    }

    /**
     * Configures the TalonFX motor settings.
     */
    private void configMotors() {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // motor.optimizeBusUtilization();
        // motor.getTorqueCurrent().setUpdateFrequency(50);

        motor.getConfigurator().apply(config);
    }

    /**
     * Configures the TalonFX encoder settings.
     */
    private void configSensors() {
        
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = 0.456;
        
        encoder.getConfigurator().apply(config);
    }

    /**
     * Returns the climber absolute position in degrees.
     */
    private double getAngle() {
        return (encoder.getAbsolutePosition().getValueAsDouble() * 360.0);
    }    

    /**
     * Runs the climber at a given current output.
     * @param current Current output in Amps
     */
    public void runClimber(double current) {
        motor.setControl(torqueCurrentFOC.withOutput(current));
    }

    /**
     * Runs the climber at a given current output.
     * @param output Speed output in duty cycle
     */
    public void runClimberVariable(double output) {
        motor.setControl(dutyCycleOut.withOutput(output));
    }

    @Override
    public void periodic() {

      SmartDashboard.putNumber("Climber: Angle", getAngle());
      SmartDashboard.putNumber("Climber: Current", motor.getTorqueCurrent().getValueAsDouble());
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
     * Command to move the climber to starting position.
     */
    public Command rotateClimberToStartingPositionCommand() {
        return rotateClimberCommand(PREPARE_CURRENT).until(() -> getAngle() <= 5).andThen(stopClimberCommand());
    }

    /**
     * Command to move the climber to climbing position.
     */
    public Command rotateClimberToAttachingPositionCommand() {
        return rotateClimberCommand(-PREPARE_CURRENT).until(() -> getAngle() >= 170).andThen(stopClimberCommand());
    }

    /**
     * Command to move the climber to lifting position.
     */
    public Command rotateClimberToClimbedPositionCommand() {
        return rotateClimberCommand(CLIMB_CURRENT).until(() -> getAngle() <= 90).andThen(stopClimberCommand());
    }

}