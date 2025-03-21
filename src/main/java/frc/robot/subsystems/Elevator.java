package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {

    public static final double BOTTOM_POSITION = 0.0;
    public static final double TOP_POSITION = 88.0;

    public static final double POSITION_CORAL_L1 = 18.0;
    public static final double POSITION_CORAL_L2 = 35.5;
    public static final double POSITION_CORAL_L3 = 52.0;
    public static final double POSITION_CORAL_L4 = 83.5;

    public static final double POSITION_ALGAE_GROUND = 3.0;
    public static final double POSITION_ALGAE_LOW = 35.5;
    public static final double POSITION_ALGAE_HIGH = 45.0;
    public static final double POSITION_ALGAE_BARGE = 86.0;

    public static final double STAGE_1 = 24.93;
    public static final double STAGE_2 = 59.3;

    private static final int MASTER_ID = 17;
    private static final int FOLLOWER_ID = 18;
    private static final int LIM_SWITCH_ID = 1;

    private static final double TOLERENCE = 1.0;

    private final TalonFX masterMotor;
    private final TalonFX followerMotor;

    private final DigitalInput bottomLimitSwitch;

    private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final DutyCycleOut dutyCycleOut;

    private final Debouncer limitSwitchDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kRising); // 100ms debounce

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
        torqueCurrentFOC = new TorqueCurrentFOC(0);
        dutyCycleOut = new DutyCycleOut(0);

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
        
        config.MotionMagic.MotionMagicAcceleration = 80;  // Acceleration (ticks/sec²)
        config.MotionMagic.MotionMagicCruiseVelocity = 60; // Max velocity (ticks/sec)
        
        config.Slot0.kP = 20.0;  // Proportional Gain (response to error)
        config.Slot0.kI = 0.0;  // Integral Gain (only if you need fine corrections)
        config.Slot0.kD = 0.0;  // Derivative Gain (smooths response)
        config.Slot0.kS = 0.0000;  // Static friction compensation (helps start moving)
        config.Slot0.kV = 0.0000;  // Velocity feedforward (scales output based on velocity)
        config.Slot0.kA = 0.0000;  // Acceleration feedforward (scales output based on acceleration)
        config.Slot0.kG = 4.0000;  // Gravity compensation (counteracts elevator weight)
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        
        config.Slot1.kP = 20.0;  // Proportional Gain (response to error)
        config.Slot1.kI = 0.0;  // Integral Gain (only if you need fine corrections)
        config.Slot1.kD = 0.0;  // Derivative Gain (smooths response)
        config.Slot1.kS = 0.0000;  // Static friction compensation (helps start moving)
        config.Slot1.kV = 0.0000;  // Velocity feedforward (scales output based on velocity)
        config.Slot1.kA = 0.0000;  // Acceleration feedforward (scales output based on acceleration)
        config.Slot1.kG = 7.0000;  // Gravity compensation (counteracts elevator weight)
        config.Slot1.GravityType = GravityTypeValue.Elevator_Static;

        config.Slot2.kP = 20.0;  // Proportional Gain (response to error)
        config.Slot2.kI = 0.0;  // Integral Gain (only if you need fine corrections)
        config.Slot2.kD = 0.0;  // Derivative Gain (smooths response)        
        config.Slot2.kS = 0.0000;  // Static friction compensation (helps start moving)
        config.Slot2.kV = 0.0000;  // Velocity feedforward (scales output based on velocity)
        config.Slot2.kA = 0.0000;  // Acceleration feedforward (scales output based on acceleration)
        config.Slot2.kG = 12.0000;  // Gravity compensation (counteracts elevator weight)
        config.Slot2.GravityType = GravityTypeValue.Elevator_Static;

        masterMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);

        followerMotor.setControl(new Follower(masterMotor.getDeviceID(), true));
    }

    private int getSlotFromPosition() {
        double currentPosition = masterMotor.getPosition().getValueAsDouble();
        if (currentPosition <= STAGE_1) {
            return 0;
        } else if (currentPosition <= STAGE_2) {
            return 1;
        } else { // STAGE_3
            return 2;
        }
    }

    private void zeroElevator() {
        boolean debouncedLimitSwitch = limitSwitchDebouncer.calculate(bottomLimitSwitch.get());
        if (debouncedLimitSwitch && !hasResetZero) {
            masterMotor.setPosition(0);
            followerMotor.setPosition(0);
            hasResetZero = true;  // Prevent constant resetting
        } else if (!bottomLimitSwitch.get()) {
            hasResetZero = false;  // Reset the flag when the switch is not pressed
        }
    }

    public double getPosition() {
        return masterMotor.getPosition().getValueAsDouble();
    }

    public double getCurrent() {
        return masterMotor.getTorqueCurrent().getValueAsDouble();
    }

    /**
     * Runs the elevator at a given current output.
     *
     * @param current Current output in Amps
     */
    public void runElevator(double current) {
        masterMotor.setControl(torqueCurrentFOC.withOutput(current));
    }

    /**
     * Runs the elevator at a given current output.
     *
     * @param output Speed output in duty cycle
     */
    public void runElevatorVariable(double output) {
        masterMotor.setControl(dutyCycleOut.withOutput(output));
    }

    /**
     * Holds the elevator at its current position.
     */
    public void holdPosition() {
        // Do nothing if elevator at bottom
        if (masterMotor.getPosition().getValueAsDouble() <= 0.1) {
            masterMotor.setControl(torqueCurrentFOC.withOutput(0));
            // Apply holding current otherwise
        } else {
            masterMotor.setControl(motionMagicTorqueCurrentFOC
                    .withSlot(getSlotFromPosition())
                    .withPosition(masterMotor.getPosition().getValueAsDouble()));
        }
    }

    /**
     * Moves the elevator to a specified position using Motion Magic with Torque FOC.
     *
     * @param targetPosition Target position in motor shaft rotations.
     */
    public void setElevatorPosition(double targetPosition) {
        // Ensure target is within soft limits
        target = Math.max(BOTTOM_POSITION, Math.min(TOP_POSITION, targetPosition));

        // Apply Motion Magic control with FOC
        masterMotor.setControl(motionMagicTorqueCurrentFOC
            .withPosition(target)
            .withSlot(getSlotFromPosition()));
    }

    @Override
    public void periodic() {
        zeroElevator();
        SmartDashboard.putBoolean("Elevator at Bottom", bottomLimitSwitch.get());
        SmartDashboard.putNumber("Elevator: Position", getPosition());
        SmartDashboard.putNumber("Elevator: Current", getCurrent());
    }

    /**
     * Command to move the elevator up.
     *
     * @param rotation The current to run the elevator in Amps
     */
    public Command jogElevatorUpCommand(double rotation) {
        return new FunctionalCommand(
                () -> target = rotation + masterMotor.getPosition().getValueAsDouble(),
                () -> setElevatorPosition(target),
                (interrupted) -> runElevator(0),
                this::isOnTarget
        );
    }

    /**
     * Command to move the elevator up.
     *
     * @param rotation The current to run the elevator in Amps
     */
    public Command jogElevatorDownCommand(double rotation) {
        return new FunctionalCommand(
                () -> target = masterMotor.getPosition().getValueAsDouble() - rotation,
                () -> setElevatorPosition(target),
                (interrupted) -> runElevator(0),
                this::isOnTarget
        );
    }

    /**
     * Command to stop the elevator.
     */
    public Command stopElevatorCommand() {
        return new InstantCommand(() -> runElevator(0), this);
    }

    /**
     * Command to move the elevator.
     *
     * @param output The current to run the elevator in Amps
     */
    public Command moveElevatorVariableCommand(DoubleSupplier output) {
        return new RunCommand(() -> runElevatorVariable(output.getAsDouble()), this)
            .finallyDo(interrupted -> runElevator(0));
    }

    /**
     * Command to move the elevator to the set position.
     *
     * @param position The position to move the elevator to in rotations
     */
    public Command moveElevatorToPositionCommand(double position) {
        return new FunctionalCommand(
                () -> {},
                () -> setElevatorPosition(position),
                (interrupted) -> runElevator(0),
                this::isOnTarget
        );
    }

    public boolean isOnTarget() {
        return Math.abs(masterMotor.getPosition().getValueAsDouble() - target) < TOLERENCE;
    }

    /**
     * Command to hold the elevator at the set position.
     */
    public Command holdElevatorPositionCommand() {
        return new RunCommand(this::holdPosition, this);
    }

}
