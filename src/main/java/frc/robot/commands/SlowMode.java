package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class SlowMode extends Command{
    private final ClimbSubsystem climbSub;

    public SlowMode(ClimbSubsystem climbSub) {
        this.climbSub = climbSub;
        addRequirements(climbSub);
    }

    @Override
    public void initialize() {
        climbSub.climbMode = 0;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
