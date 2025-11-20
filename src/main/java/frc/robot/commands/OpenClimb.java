package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OpenClimb extends command{
    private final ClimbSubsystem climbSub;

    public OpenClimb(ClimbSubsystem climbSub) {
        this.climbSub = climbSub;

        addRequirements(climbSub);
    }

    @Override
    public void execute(){
        climbSub.openClimbFast();
    }

    @Override
    public void isFinished(){
        return false;
    }
}
