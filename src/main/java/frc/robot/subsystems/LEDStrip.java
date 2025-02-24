package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
  
  private static final int PWM_PORT = 1;
  private static final int NUM_LEDS = 66;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  public LEDStrip() {
    this(PWM_PORT, NUM_LEDS);
  }

  public LEDStrip(int pwmPort, int numLEDS) {
    m_led = new AddressableLED(pwmPort);
    m_ledBuffer = new AddressableLEDBuffer(numLEDS);
    m_led.setLength(m_ledBuffer.getLength());
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
    // Create an LED pattern that displays a red-to-blue gradient, breathing at a 2 second period (0.5 Hz)
    // LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kOrange, Color.kNavy);
    // LEDPattern base = LEDPattern.rainbow(255, NUM_LEDS)
    // LEDPattern pattern = base.breathe(Seconds.of(2));
    // pattern = base.atBrightness(Percent.of(25));
    LEDPattern red = LEDPattern.solid(Color.kRed).atBrightness(Percent.of(25));
    
    // Apply the LED pattern to the data buffer
    red.applyTo(m_ledBuffer);

    // Write the data to the LED strip
    m_led.setData(m_ledBuffer);
    m_led.start();
  }
  
}