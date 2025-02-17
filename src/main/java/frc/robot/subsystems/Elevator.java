package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.Follower;

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
    
    private final TalonFX masterMotor;
    private final TalonFX followerMotor;

    public Elevator() {
        this(MASTER_ID, FOLLOWER_ID);
    }
    
    public Elevator(int masterID, int followerID) {
        masterMotor = new TalonFX(masterID);
        followerMotor = new TalonFX(followerID);
        
        configMotors();
    }
    
    private void configMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();        
        masterMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);
        
        // Ensure follower motor runs in the opposite direction of the master motor
        followerMotor.setControl(new Follower(masterMotor.getDeviceID(), true));
    }
    
    public void runElevator(double current) {
        masterMotor.setControl(new TorqueCurrentFOC(current));
    }
    
    public void stop() {
        masterMotor.set(0);
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
    
    public Command moveUpCommand(double current) {
        return new RunCommand(() -> runElevator(current), this).finallyDo(interrupted -> stop());
    }
    
    public Command moveDownCommand(double current) {
        return new RunCommand(() -> runElevator(-current), this).finallyDo(interrupted -> stop());
    }
}
