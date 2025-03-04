package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase { 
    
    private static final int MASTER_ID = 17;
    private static final int FOLLOWER_ID = 18;
    private static final int LIM_SWITCH_ID = 1;

    private static final double BOTTOM_POSITION = 0.0;
    private static final double TOP_POSITION = 100000.0;

    private static final double POSITION_1 = 10;
    private static final double POSITION_2 = 20;
    private static final double POSITION_3 = 30;
    private static final double POSITION_4 = 40;
    private static final double POSITION_5 = 50;


    
    private final TalonFX masterMotor;
    private final TalonFX followerMotor;

    private final DigitalInput bottomLimitSwitch;

    private final MotionMagicTorqueCurrentFOC motionMagicControl;

    private double lastPosition = 0.0;

    public Elevator() {
        this(MASTER_ID, FOLLOWER_ID, LIM_SWITCH_ID);
    }
    
    public Elevator(int masterID, int followerID, int limSwitchID) {
        masterMotor = new TalonFX(masterID, "hyperbus");
        followerMotor = new TalonFX(followerID, "hyperbus");

        bottomLimitSwitch = new DigitalInput(limSwitchID);
        motionMagicControl = new MotionMagicTorqueCurrentFOC(0);

        configMotors();
    }
    
    /**
     * Configures the TalonFX settings.
     */
    private void configMotors() {      

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TOP_POSITION;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = BOTTOM_POSITION;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.TorqueCurrent.TorqueNeutralDeadband = 0.01; // Prevents small unintended movements
        config.MotorOutput.DutyCycleNeutralDeadband = 0.01;
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5; // 0.5s to reach full speed in open-loop mode
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.3; // 0.3s in closed-loop mode

        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40;
        config.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60; // Limit stator current to 60A
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // ✅ Set Motion Magic parameters
        config.MotionMagic.MotionMagicAcceleration = 5000;  // Acceleration (ticks/sec²)
        config.MotionMagic.MotionMagicCruiseVelocity = 8000; // Max velocity (ticks/sec)

        // ✅ Tune PID values (adjust as needed)
        config.Slot0.kP = 0.1;  // Proportional Gain (response to error)
        config.Slot0.kI = 0.0;  // Integral Gain (only if you need fine corrections)
        config.Slot0.kD = 0.0;  // Derivative Gain (smooths response)

        // ✅ Set Feedforward Gains (for smooth motion)
        config.Slot0.kS = 0.05;  // Static friction compensation (helps start moving)
        config.Slot0.kV = 0.12;  // Velocity feedforward (scales output based on velocity)
        config.Slot0.kA = 0.01;  // Acceleration feedforward (scales output based on acceleration)
        config.Slot0.kG = 0.08;  // Gravity compensation (counteracts elevator weight)

        // ✅ Set Gravity Type (for elevators)
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        masterMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);
        
        // Ensure follower motor runs in the opposite direction of the master motor
        followerMotor.setControl(new Follower(masterMotor.getDeviceID(), true));

        masterMotor.setPosition(0.0);  // Set initial encoder position
        followerMotor.setPosition(0.0);
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

    /**
     * Holds the elevator at its current position.
     */
    public void holdPosition() {
        motionMagicControl.withPosition(lastPosition);
        masterMotor.setControl(motionMagicControl);
    }

    /**
     * Moves the elevator to a specified position using Motion Magic with Torque FOC.
     * @param targetPosition Target position in motor shaft rotations.
     */
    public void setElevatorPosition(double targetPosition) {
        // Ensure target is within soft limits
        targetPosition = Math.max(BOTTOM_POSITION, Math.min(TOP_POSITION, targetPosition));

        // Apply Motion Magic control with FOC
        motionMagicControl.withPosition(targetPosition);
        masterMotor.setControl(motionMagicControl);

        lastPosition = targetPosition;
    }
    
    @Override
    public void periodic() {

        SmartDashboard.putBoolean("Bottom Limit Switch", bottomLimitSwitch.get());

        SmartDashboard.putNumber("Elevator Position", masterMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Current Master", masterMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Current Follower", followerMotor.getStatorCurrent().getValueAsDouble());
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

    /**
     * Command to move the elevator to the set position.
     * @param position The position to move the elevator to in rotations
     */
    public Command moveToPositionCommand(double position) {
        return new InstantCommand(() -> setElevatorPosition(position), this);
    }

    /**
     * Command to hold the elevator at the set position.
     */
    public Command holdElevatorPositionCommand() {
        return new RunCommand(() -> holdPosition(), this);
    }

}
