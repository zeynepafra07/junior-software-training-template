package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class FastMode extends Command{
    private final ClimbSubsystem climbSub;

    public FastMode(ClimbSubsystem climbSub) {
        this.climbSub = climbSub;
        addRequirements(climbSub);
    }

    @Override
    public void initialize() {
        climbSub.climbMode = 1;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
