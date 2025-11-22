package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.Constants;

public class NormalMode extends CommandBase{
    private final ClimbSubsystem climbSub;

    public NormalMode(ClimbSubsystem climbSub) {
        this.climbSub = climbSub;
        addRequirements(climbSub);
    }

    @Override
    public void initialize() {
        climbSub.climbMode = 2;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
