package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase { 
    
    private static final int MASTER_ID = 17;
    private static final int FOLLOWER_ID = 18;
    private static final int LIM_SWITCH_ID = 1;

    public static final double BOTTOM_POSITION = 0.0;
    public static final double TOP_POSITION = 88.0;

    public static final double POSITION_CORAL_L1 = 17.0;
    public static final double POSITION_CORAL_L2 = 34.5;
    public static final double POSITION_CORAL_L3 = 51.0;
    public static final double POSITION_CORAL_L4 = 81.5;
    public static final double POSITION_ALGAE_LOW = 26.5;
    public static final double POSITION_ALGAE_HIGH = 41.0;
    public static final double POSITION_ALGAE_BARGE = 85.0; // or 88 with carry algae position

    private static final double STAGE_1 = 24.93;
    private static final double STAGE_2 = 59.3;
    private static final double STAGE_3 = 88.0;
    
    private final TalonFX masterMotor;
    private final TalonFX followerMotor;

    private final DigitalInput bottomLimitSwitch;

    private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentFOC;

    private boolean hasResetZero = false;

    private double target = 0.0;

    public Elevator() {
        this(MASTER_ID, FOLLOWER_ID, LIM_SWITCH_ID);
    }
    
    public Elevator(int masterID, int followerID, int limSwitchID) {
        masterMotor = new TalonFX(masterID, "hyperbus");
        followerMotor = new TalonFX(followerID, "hyperbus");

        bottomLimitSwitch = new DigitalInput(limSwitchID);
        motionMagicTorqueCurrentFOC = new MotionMagicTorqueCurrentFOC(0);

        configMotors();

        setDefaultCommand(holdElevatorPositionCommand());
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

        // config.TorqueCurrent.TorqueNeutralDeadband = 0.01; // Prevents small unintended movements
        // config.MotorOutput.DutyCycleNeutralDeadband = 0.01;
        // config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5; // 0.5s to reach full speed in open-loop mode
        // config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.3; // 0.3s in closed-loop mode

        // config.CurrentLimits.SupplyCurrentLimit = 40;
        // config.CurrentLimits.SupplyCurrentLowerLimit = 40;
        // config.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        // config.CurrentLimits.SupplyCurrentLimitEnable = true;
        // config.CurrentLimits.StatorCurrentLimit = 60; // Limit stator current to 60A
        // config.CurrentLimits.StatorCurrentLimitEnable = true;

        // ✅ Set Motion Magic parameters
        config.MotionMagic.MotionMagicAcceleration = 80;  // Acceleration (ticks/sec²)
        config.MotionMagic.MotionMagicCruiseVelocity = 60; // Max velocity (ticks/sec)

        // ✅ Tune PID values (adjust as needed)
        config.Slot0.kP = 20.0;  // Proportional Gain (response to error)
        config.Slot0.kI = 0.0;  // Integral Gain (only if you need fine corrections)
        config.Slot0.kD = 0.0;  // Derivative Gain (smooths response)
        // ✅ Set Feedforward Gains (for smooth motion)
        config.Slot0.kS = 0.0000;  // Static friction compensation (helps start moving)
        config.Slot0.kV = 0.0000;  // Velocity feedforward (scales output based on velocity)
        config.Slot0.kA = 0.0000;  // Acceleration feedforward (scales output based on acceleration)
        config.Slot0.kG = 4.0000;  // Gravity compensation (counteracts elevator weight)
        // ✅ Set Gravity Type (for elevators)
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        // ✅ Tune PID values (adjust as needed)
        config.Slot1.kP = 20.0;  // Proportional Gain (response to error)
        config.Slot1.kI = 0.0;  // Integral Gain (only if you need fine corrections)
        config.Slot1.kD = 0.0;  // Derivative Gain (smooths response)
        // ✅ Set Feedforward Gains (for smooth motion)
        config.Slot1.kS = 0.0000;  // Static friction compensation (helps start moving)
        config.Slot1.kV = 0.0000;  // Velocity feedforward (scales output based on velocity)
        config.Slot1.kA = 0.0000;  // Acceleration feedforward (scales output based on acceleration)
        config.Slot1.kG = 7.0000;  // Gravity compensation (counteracts elevator weight)
        // ✅ Set Gravity Type (for elevators)
        config.Slot1.GravityType = GravityTypeValue.Elevator_Static;

        // ✅ Tune PID values (adjust as needed)
        config.Slot2.kP = 20.0;  // Proportional Gain (response to error)
        config.Slot2.kI = 0.0;  // Integral Gain (only if you need fine corrections)
        config.Slot2.kD = 0.0;  // Derivative Gain (smooths response)
        // ✅ Set Feedforward Gains (for smooth motion)
        config.Slot2.kS = 0.0000;  // Static friction compensation (helps start moving)
        config.Slot2.kV = 0.0000;  // Velocity feedforward (scales output based on velocity)
        config.Slot2.kA = 0.0000;  // Acceleration feedforward (scales output based on acceleration)
        config.Slot2.kG = 12.0000;  // Gravity compensation (counteracts elevator weight)
        // ✅ Set Gravity Type (for elevators)
        config.Slot2.GravityType = GravityTypeValue.Elevator_Static;

        masterMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);
        
        // Ensure follower motor runs in the opposite direction of the master motor
        followerMotor.setControl(new Follower(masterMotor.getDeviceID(), true));
    }

    private int getSlotFromPosition() {
        double currentPosition = masterMotor.getPosition().getValueAsDouble();
        if (currentPosition <= STAGE_1) {
            return 0;
        } else if (currentPosition <= STAGE_2) {
            return 1;
        } else { 
            return 2;
        }
    }

    private void zeroElevator() {
        if (bottomLimitSwitch.get() && !hasResetZero) {
            // Set the motor's position to zero
            masterMotor.setPosition(0);
            followerMotor.setPosition(0);
            hasResetZero = true;  // Prevent constant resetting
        } else if (!bottomLimitSwitch.get()) {
            hasResetZero = false;  // Reset the flag when the switch is not pressed
        }
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
        masterMotor.setControl(new DutyCycleOut(output).withIgnoreHardwareLimits(true));
    }

    /**
     * Holds the elevator at its current position.
     */
    public void holdPosition() {
        // do nothing if elevator at bottom
        if (masterMotor.getPosition().getValueAsDouble() <= 0.1) {
            masterMotor.setControl( new TorqueCurrentFOC(0));
        // apply holding current otherwise
        } else {
            masterMotor.setControl(motionMagicTorqueCurrentFOC
                .withSlot(getSlotFromPosition()).withPosition(masterMotor.getPosition().getValueAsDouble()));
        }        
    }

    /**
     * Moves the elevator to a specified position using Motion Magic with Torque FOC.
     * @param targetPosition Target position in motor shaft rotations.
     */
    public void setElevatorPosition(double targetPosition) {
        // Ensure target is within soft limits
        targetPosition = Math.max(BOTTOM_POSITION, Math.min(TOP_POSITION, targetPosition));

        // Apply Motion Magic control with FOC
        masterMotor.setControl(new MotionMagicTorqueCurrentFOC(targetPosition).withSlot(getSlotFromPosition()));
    }
    
    @Override
    public void periodic() {

        zeroElevator();
        SmartDashboard.putBoolean("Bottom Limit Switch", bottomLimitSwitch.get());

        // target = masterMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("target", target);

        SmartDashboard.putNumber("Elevator Position Master", masterMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Position Follower", followerMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Current Master", masterMotor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Current Follower", followerMotor.getTorqueCurrent().getValueAsDouble());        
    }
    
    /**
     * Command to move the elevator up.
     * @param rotations The current to run the elevator in Amps
     */
    public Command moveUpCommand(double rotation) {
        return new FunctionalCommand(
            () -> {target = rotation + masterMotor.getPosition().getValueAsDouble();},
            () -> setElevatorPosition(rotation),
            (interrupted)-> runElevator(0),
            () -> Math.abs(masterMotor.getPosition().getValueAsDouble() - target) < 0.01
        );
    }
    
    /**
     * Command to move the elevator down.
     * @param rotations The current to run the elevator in Amps
     */
    public Command moveDownCommand(double rotations) {
        double target = masterMotor.getPosition().getValueAsDouble() - rotations;
        return new RunCommand(() -> setElevatorPosition(target), this);
            // .until(() -> Math.abs(masterMotor.getPosition().getValueAsDouble() - target) < 0.05)
            // .finallyDo(interrupted -> runElevator(0));
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
        return new FunctionalCommand(
            () -> {},
            () -> setElevatorPosition(target),
            (interrupted)-> runElevator(0),
            () -> Math.abs(masterMotor.getPosition().getValueAsDouble() - position) < 0.01
        );
    }

    /**
     * Command to hold the elevator at the set position.
     */
    public Command holdElevatorPositionCommand() {
        return new RunCommand(() -> holdPosition(), this);
    }

}
