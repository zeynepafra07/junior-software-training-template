package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class CloseClimb extends Command{
    private final ClimbSubsystem climbSub;
    private double startTime;

    public CloseClimb(ClimbSubsystem climbSub) {
        this.climbSub = climbSub;

        addRequirements(climbSub);
    }

    @Override
    public void initialize(){
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        if (Timer.getFPGATimestamp() - startTime < 1.0) {
            climbSub.closeClimb();
        } else {
            climbSub.holdClimb();
        }
    }

    @Override
    public void end(boolean interrupted) {
        climbSub.holdClimb();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
