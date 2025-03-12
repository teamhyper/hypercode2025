package frc.robot.commands.ledCommands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStrip;

public class BlinkLEDCommand extends Command {

    private static final double MAGNITUDE = 25;
  
    private final LEDStrip ledStrip;
    private final Timer timer;
    private final LEDPattern redPattern;
    private final LEDPattern offPattern;    

    private double blinkInterval;
    private boolean isOn = true;

  public BlinkLEDCommand(LEDStrip ledStrip, Color color, double interval) {
    this.ledStrip = ledStrip;
    this.timer = new Timer();
    this.redPattern = LEDPattern.solid(color).atBrightness(Units.Percent.of(MAGNITUDE));
    this.offPattern = LEDPattern.solid(Color.kBlack);
    this.blinkInterval = interval;

    addRequirements(ledStrip);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    ledStrip.setPattern(redPattern);
  }

  @Override
  public void execute() {
    if (timer.hasElapsed(blinkInterval)) {
      isOn = !isOn;
      ledStrip.setPattern(isOn ? redPattern : offPattern);
      timer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    // ledStrip.setRedColor(); // Restore default solid red
  }
}
