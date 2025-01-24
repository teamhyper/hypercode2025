package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToAprilTagCommand extends Command {

    public DriveToAprilTagCommand(VisionSubsystem visionSubsystem, CommandSwerveDrivetrain drivetrain) {
        addRequirements(visionSubsystem, drivetrain);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}