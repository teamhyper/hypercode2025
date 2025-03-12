package frc.robot.joysticks;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Wrapper for the Apem HF45S10U Hall Effect Joystick.
 * Extends WPILib's CommandJoystick for easy integration.
 */
public class ApemHF45Joystick extends CommandJoystick {

    // Axis mappings
    private static final int X_AXIS = 0;  // Left-Right movement
    private static final int Y_AXIS = 1;  // Forward-Backward movement
    private static final int Z_ROTATION = 2; // Twist (if available)
    private static final int LEFT_BUTTON = 1; // Pushbutton (if applicable)
    private static final int RIGHT_BUTTON = 2; // Pushbutton (if applicable)

    /**
     * Constructor for the Apem HF45S10U Joystick.
     * 
     * @param port USB port for the joystick (e.g., 0 for first joystick).
     */
    public ApemHF45Joystick(int port) {
        super(port);
    }

    // Axis getters
    public double getXAxis() {
        return getRawAxis(X_AXIS);
    }

    public double getYAxis() {
        return getRawAxis(Y_AXIS);
    }

    public double getZRotation() {
        return getRawAxis(Z_ROTATION);
    }

    // Button wrapper (for pushbutton if available)
    public Trigger leftButton() {
        return leftButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger leftButton(EventLoop loop) {
        return button(LEFT_BUTTON, loop);
    }

    public Trigger rightButton() {
        return rightButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger rightButton(EventLoop loop) {
        return button(RIGHT_BUTTON, loop);
    }
}
