package frc.robot.commands.ledCommands;

import edu.wpi.first.wpilibj.LEDPattern;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStrip;

public class SetLEDPatternCommand extends Command {
    
    private static final Distance kLedSpacing = Meters.of(1 / 60.0);

    private final LEDStrip ledStrip;
    private final LEDPattern rainbowPattern;
    private final LEDPattern scrollingRainbowPattern;

  public SetLEDPatternCommand(LEDStrip ledStrip) {
    this.ledStrip = ledStrip;
    rainbowPattern = LEDPattern.rainbow(255, 64);
    scrollingRainbowPattern = rainbowPattern.scrollAtAbsoluteSpeed(InchesPerSecond.of(50), kLedSpacing);

    addRequirements(ledStrip);
  }

  @Override
  public void initialize() {
    ledStrip.setPattern(scrollingRainbowPattern);
  }

  @Override
  public void execute() {
    ledStrip.setPattern(scrollingRainbowPattern);
  }

  @Override
  public void end(boolean interrupted) {
    // ledStrip.setRedColor(); // Restore default solid red
  }
}
