package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ratchet extends SubsystemBase{

    private static final int RATCHET_PWM_ID = 1;
    private static final double RATCHET_LOCK_POSITION = 1.0; // TODO
    private static final double RATCHET_UNLOCK_POSITION  = 0.0; // TODO

    private final Servo s_ratchet;

    public Ratchet() {
        this(RATCHET_PWM_ID);
    }

    public Ratchet(int ratchetPwmID) {
        s_ratchet = new Servo(ratchetPwmID);
    }

    /**
     * Returns boolean of last commanded ratchet position.
     */
    private boolean getRatchetLocked() {
        return s_ratchet.get() == RATCHET_LOCK_POSITION;
    }
    
    /**
     * Set ratchet servo position.
     * @param position Servo position between 0.0 to 1.0
     */
    public void setRatchet(double position) {
        s_ratchet.set(position);
    }

    @Override
    public void periodic() {
      SmartDashboard.putBoolean("Ratchet Locked", getRatchetLocked());
    }
    
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
