package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivetrain;

enum Side {left, right};

public class MoveToTagCommand extends Command{
    private final Vision vision;
    private final Drivetrain drivetrain;
    private final Side side;

    public MoveToTagCommand(Vision vision, Drivetrain drivetrain, Side side) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.side = side;
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

    
}
