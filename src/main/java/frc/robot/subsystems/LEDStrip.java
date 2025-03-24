package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class LEDStrip extends SubsystemBase {
  
    private static final int PWM_PORT = 0;
    private static final int NUM_LEDS = 66;
    private static final double BLINK_INTERVAL = 0.2;

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private final Timer timer;
    private boolean isOn;
    

    public LEDStrip() {
        this(PWM_PORT, NUM_LEDS);
    }

    public LEDStrip(int pwmPort, int numLEDS) {
        m_led = new AddressableLED(pwmPort);
        m_ledBuffer = new AddressableLEDBuffer(numLEDS);
        timer = new Timer();

        init();
    }

    private void init() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();
        isOn = false;
        setColor(Color.kRed);
    }

    public void setColor(Color color) {
        LEDPattern.solid(color).atBrightness(Percent.of(25)).applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    public Command setColorCommand(Color color) {
        return new RunCommand(() -> setColor(color), this);
    }

    public Command setColorAndBlinkCommand(Color color) {
        return new FunctionalCommand(
            () -> {
                timer.reset();
                timer.start();
            }, 
            () -> {
                if (timer.hasElapsed(BLINK_INTERVAL)) {
                    if (isOn) {
                        setColor(Color.kBlack);
                        
                    } else {
                        setColor(color);
                    }
                    isOn = !isOn;
                    timer.reset();
                }
            }, 
            interrupted -> {
                timer.stop();
                isOn = false;
            }, 
            () -> false, 
            this);
    }
}
