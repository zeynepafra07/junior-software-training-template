package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class OpenClimb extends Command{
    private final ClimbSubsystem climbSub;
    private final ElevatorSubsystem elevatorSub;

    public OpenClimb(ClimbSubsystem climbSub, ElevatorSubsystem elevatorSub) {
        this.climbSub = climbSub;
        this.elevatorSub = elevatorSub;

        addRequirements(climbSub, elevatorSub);
    }

    @Override
    public void initialize(){
        elevatorSub.reachPosition(12).withTimeout(3);
    }

    @Override
    public void execute(){
        climbSub.openClimb();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
