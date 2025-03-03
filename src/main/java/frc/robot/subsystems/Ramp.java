package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ramp extends SubsystemBase{

    private static final int RAMP_PWM_ID = 2;

    private final Servo s_ramp;
    private final Timer timer;

    public Ramp() {
        this(RAMP_PWM_ID);
    }

    public Ramp(int rampPwmID) {
        s_ramp = new Servo(rampPwmID);
        timer = new Timer();
    }
    
    /**
     * Set ratchet servo output.
     * @param position Servo output between 0.0 to 1.0
     */
    public void runServo(double output) {
        s_ramp.set(output);
    }

    @Override
    public void periodic() {
    }

    /**
     * Command to run the servo for a set duration.
     * @param speed Servo speed (0 to 1.0).
     * @param duration Time in seconds.
     * @return A command that runs the servo for the given duration.
     */
    public Command runServoForTime(double speed, double duration) {
        return new Command() {
            @Override
            public void initialize() {
                timer.reset();
                timer.start();
                runServo(speed);
            }

            @Override
            public void execute() {
                // No need to do anything, just waiting for time
            }

            @Override
            public boolean isFinished() {
                return timer.get() >= duration;
            }

            @Override
            public void end(boolean interrupted) {
                runServo(0.0); // Stop the servo
                timer.stop();
            }
        }.withTimeout(duration); // Ensure timeout safety
    }

}
