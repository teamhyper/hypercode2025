package frc.robot.commands.ledCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStrip;

public class BlinkLEDCommand extends Command {
  
  private final LEDStrip ledStrip;
  private final Timer timer;
  private final LEDPattern redPattern;
  private final LEDPattern offPattern;
  private static final double BLINK_INTERVAL = 0.3;

  private boolean isOn = true;

  public BlinkLEDCommand(LEDStrip ledStrip) {
    this.ledStrip = ledStrip;
    this.timer = new Timer();
    this.redPattern = LEDPattern.solid(Color.kRed).atBrightness(Units.Percent.of(25));
    this.offPattern = LEDPattern.solid(Color.kBlack);
    
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
    if (timer.hasElapsed(BLINK_INTERVAL)) {
      isOn = !isOn;
      ledStrip.setPattern(isOn ? redPattern : offPattern);
      timer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    ledStrip.setRedColor(); // Restore default solid red
  }
}
