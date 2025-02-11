package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Wrapper for the VKB Gladiator NXT EVO Space Combat Edition (Left-Hand)
 * using WPILib's CommandJoystick-style API with overloaded functions.
 */
public class VKBGladiatorLHJoystick extends CommandJoystick {
    
    // Button mappings based on VKB Gladiator NXT EVO layout
    private static final int TRIGGER_PRIMARY_BUTTON = 1;
    private static final int TRIGGER_SECONDARY_BUTTON = 1;
    private static final int RED_BUTTON = 2;
    private static final int SIDE_BUTTON = 3;
    private static final int THUMB_BUTTON = 4;
    private static final int PINKY_BUTTON = 5;
    private static final int INNER_HAT_UP = 6;
    private static final int INNER_HAT_DOWN = 7;
    private static final int INNER_HAT_LEFT = 8;
    private static final int INNER_HAT_RIGHT = 9;
    private static final int OUTER_HAT_UP = 6;
    private static final int OUTER_HAT_DOWN = 7;
    private static final int OUTER_HAT_LEFT = 8;
    private static final int OUTER_HAT_RIGHT = 9;
    private static final int LOWER_HAT_UP = 6;
    private static final int LOWER_HAT_DOWN = 7;
    private static final int LOWER_HAT_LEFT = 8;
    private static final int LOWER_HAT_RIGHT = 9;
    private static final int F2_BUTTON = 10;
    private static final int F1_BUTTON = 11;
    private static final int F3_BUTTON = 11;
    private static final int SW1_BUTTON = 11;
    private static final int EN1_BUTTON = 11;

    // Axis mappings
    private static final int X_AXIS = 0;
    private static final int Y_AXIS = 1;
    private static final int Z_ROTATION = 2;       // Twist axis
    private static final int THROTTLE_AXIS = 3;    // Throttle Potentiometer

    /**
     * Constructor for the VKB Gladiator Joystick.
     * 
     * @param port USB port for the joystick (e.g., 0 for first joystick).
     */
    public VKBGladiatorLHJoystick(int port) {
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

    public double getThrottle() {
        return getRawAxis(THROTTLE_AXIS);
    }

    // Overloaded button getters

    public Trigger triggerPrimary() {
        return triggerPrimary(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger triggerPrimary(EventLoop loop) {
        return button(TRIGGER_PRIMARY_BUTTON, loop);
    }

    public Trigger triggerSecondary() {
        return triggerSecondary(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger triggerSecondary(EventLoop loop) {
        return button(TRIGGER_SECONDARY_BUTTON, loop);
    }

    public Trigger redButton() {
        return redButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger redButton(EventLoop loop) {
        return button(RED_BUTTON, loop);
    }

    public Trigger sideButton() {
        return sideButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger sideButton(EventLoop loop) {
        return button(SIDE_BUTTON, loop);
    }

    public Trigger thumbButton() {
        return thumbButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger thumbButton(EventLoop loop) {
        return button(THUMB_BUTTON, loop);
    }

    public Trigger pinkyButton() {
        return pinkyButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger pinkyButton(EventLoop loop) {
        return button(PINKY_BUTTON, loop);
    }

    public Trigger innerHatUp() {
        return innerHatUp(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger innerHatUp(EventLoop loop) {
        return button(INNER_HAT_UP, loop);
    }

    public Trigger innerHatDown() {
        return innerHatDown(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger innerHatDown(EventLoop loop) {
        return button(INNER_HAT_DOWN, loop);
    }

    public Trigger innerHatLeft() {
        return innerHatLeft(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger innerHatLeft(EventLoop loop) {
        return button(INNER_HAT_LEFT, loop);
    }

    public Trigger innerHatRight() {
        return innerHatRight(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger innerHatRight(EventLoop loop) {
        return button(INNER_HAT_RIGHT, loop);
    }

    public Trigger outerHatUp() {
        return outerHatUp(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger outerHatUp(EventLoop loop) {
        return button(OUTER_HAT_UP, loop);
    }

    public Trigger outerHatDown() {
        return outerHatDown(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger outerHatDown(EventLoop loop) {
        return button(OUTER_HAT_DOWN, loop);
    }

    public Trigger outerHatLeft() {
        return outerHatLeft(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger outerHatLeft(EventLoop loop) {
        return button(OUTER_HAT_LEFT, loop);
    }

    public Trigger outerHatRight() {
        return outerHatRight(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger outerHatRight(EventLoop loop) {
        return button(OUTER_HAT_RIGHT, loop);
    }

    public Trigger lowerHatUp() {
        return lowerHatUp(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger lowerHatUp(EventLoop loop) {
        return button(LOWER_HAT_UP, loop);
    }

    public Trigger lowerHatDown() {
        return lowerHatDown(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger lowerHatDown(EventLoop loop) {
        return button(LOWER_HAT_DOWN, loop);
    }

    public Trigger lowerHatLeft() {
        return lowerHatLeft(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger lowerHatLeft(EventLoop loop) {
        return button(LOWER_HAT_LEFT, loop);
    }

    public Trigger lowerHatRight() {
        return lowerHatRight(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger lowerHatRight(EventLoop loop) {
        return button(LOWER_HAT_RIGHT, loop);
    }

    public Trigger f2Button() {
        return f2Button(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger f2Button(EventLoop loop) {
        return button(F2_BUTTON, loop);
    }

    public Trigger f1Button() {
        return f1Button(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger f1Button(EventLoop loop) {
        return button(F1_BUTTON, loop);
    }

    public Trigger f3Button() {
        return f3Button(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger f3Button(EventLoop loop) {
        return button(F3_BUTTON, loop);
    }

    public Trigger sw1Button() {
        return sw1Button(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger sw1Button(EventLoop loop) {
        return button(SW1_BUTTON, loop);
    }

    public Trigger en1Button() {
        return en1Button(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger en1Button(EventLoop loop) {
        return button(EN1_BUTTON, loop);
    }
}
