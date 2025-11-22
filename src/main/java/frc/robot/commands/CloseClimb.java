package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class closeClimb extends CommadBase{
    private final ClimbSubsystem climbSub;

    public closeClimb(ClimbSubsystem climbSub) {
        this.climbSub = climbSub;

        addRequirements(climbSub);
    }

    @Override
    public void execute(){
        climbSub.closeClimb();
    }

    @Override
    public void end(){
        climbSub.holdClimb();
    }

    @Override
    public void isFinished(){
        return false;
    }
}
