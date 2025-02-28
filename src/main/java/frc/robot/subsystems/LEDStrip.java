package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
  
  private static final int PWM_PORT = 1;
  private static final int NUM_LEDS = 66;
  private static final double BLINK_INTERVAL = 0.5; // Blink every 0.5 seconds

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private final Timer blinkTimer;

  private boolean isBlinkOn = true; // Track LED state for blinking

  public LEDStrip() {
    this(PWM_PORT, NUM_LEDS);
  }

  public LEDStrip(int pwmPort, int numLEDS) {
    m_led = new AddressableLED(pwmPort);
    m_ledBuffer = new AddressableLEDBuffer(numLEDS);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();

    blinkTimer = new Timer();
    blinkTimer.start(); // Start the timer
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {

    LEDPattern red = LEDPattern.solid(Color.kRed).atBrightness(Percent.of(25));

    if (RobotState.isEnabled()) {      
      if (blinkTimer.hasElapsed(BLINK_INTERVAL)) {
        isBlinkOn = !isBlinkOn; // Toggle LED state
        blinkTimer.reset(); // Reset the timer
      }

      if (isBlinkOn) {
        red.applyTo(m_ledBuffer);
      } else {
        LEDPattern.solid(Color.kBlack).applyTo(m_ledBuffer); // Turn off LEDs
      }
    } else {
      // Robot is disabled, keep LEDs solid
      red.applyTo(m_ledBuffer);
    }

    m_led.setData(m_ledBuffer);
  }
  
}