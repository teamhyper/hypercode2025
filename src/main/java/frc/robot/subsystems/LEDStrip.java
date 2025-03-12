package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
  
  private static final int PWM_PORT = 0;
  private static final int NUM_LEDS = 66;

  private static final Distance kLedSpacing = Meters.of(1 / 60.0);

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private final LEDPattern redPattern;  

  public LEDStrip() {
    this(PWM_PORT, NUM_LEDS);
  }

  public LEDStrip(int pwmPort, int numLEDS) {
    m_led = new AddressableLED(pwmPort);
    m_ledBuffer = new AddressableLEDBuffer(numLEDS);

    // Define the red color pattern at 25% brightness
    redPattern = LEDPattern.solid(Color.kRed).atBrightness(Percent.of(25));    

    init();
  }

  private void init() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();

    // Set LEDs to default red
    setRedColor();

    // Set the default command to keep LEDs red unless overridden
    setDefaultCommand(new InstantCommand(this::setRedColor, this));
  }

  public void setRedColor() {
    redPattern.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
  }

  public void setColor(Color color) {
    LEDPattern.solid(color).atBrightness(Percent.of(25)).applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
  }

  public void setPattern(LEDPattern pattern) {
    pattern.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
  }

}
