package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    
    private static final double BOTTOM_POSITION = 0.0;
    private static final double TOP_POSITION = 100000.0;
    private static final int MASTER_ID = 17;
    private static final int FOLLOWER_ID = 18;
    private static final int LIM_SWITCH_ID = 1;
    
    private final TalonFX masterMotor;
    private final TalonFX followerMotor;

    private final DigitalInput bottomLimitSwitch;

    public Elevator() {
        this(MASTER_ID, FOLLOWER_ID, LIM_SWITCH_ID) ;
    }
    
    public Elevator(int masterID, int followerID, int limSwitchID) {
        masterMotor = new TalonFX(masterID, "hyperbus");
        followerMotor = new TalonFX(followerID, "hyperbus");

        bottomLimitSwitch = new DigitalInput(limSwitchID);

        configMotors();
    }
    
    /**
     * Configures the TalonFX settings.
     */
    private void configMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // config
        //     .SoftwareLimitSwitch.ForwardSoftLimitEnable
               
        masterMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);
        
        // Ensure follower motor runs in the opposite direction of the master motor
        followerMotor.setControl(new Follower(masterMotor.getDeviceID(), true));
    }

    /**
     * Runs the elevator at a given current output.
     * @param current Current output in Amps
     */
    public void runElevator(double current) {
        masterMotor.setControl(new TorqueCurrentFOC(current));
    }

    /**
     * Runs the elevator at a given current output.
     * @param output Speed output in duty cycle
     */
    public void runElevatorVariable(double output) {
        masterMotor.setControl(new DutyCycleOut(output));
    }
    
    @Override
    public void periodic() {
        double encoderPosition = masterMotor.getPosition().getValueAsDouble();
        double currentMaster = masterMotor.getStatorCurrent().getValueAsDouble(); // Get current draw
        double currentFollower = followerMotor.getStatorCurrent().getValueAsDouble(); // Get current draw
        SmartDashboard.putNumber("Elevator Position", encoderPosition);
        SmartDashboard.putNumber("currentMaster", currentMaster);
        SmartDashboard.putNumber("currentFollower", currentFollower);
    }
    
    /**
     * Command to move the elevator up.
     * @param current The current to run the elevator in Amps
     */
    public Command moveUpCommand(double current) {
        return new RunCommand(() -> runElevator(current), this).finallyDo(interrupted -> runElevator(0));
    }
    
    /**
     * Command to move the elevator down.
     * @param current The current to run the elevator in Amps
     */
    public Command moveDownCommand(double current) {
        return new RunCommand(() -> runElevator(-current), this).finallyDo(interrupted -> runElevator(0));
    }

    /**
     * Command to stop the elevator.
     */
    public Command stopElevatorCommand() {
        return new InstantCommand(() -> runElevator(0), this);
    }

    /**
     * Command to move the elevator.
     * @param output The current to run the elevator in Amps
     */
    public Command moveVariableCommand(DoubleSupplier output) {
        return new RunCommand(() -> runElevatorVariable(output.getAsDouble()), this).finallyDo(interrupted -> runElevator(0));
    }
}
