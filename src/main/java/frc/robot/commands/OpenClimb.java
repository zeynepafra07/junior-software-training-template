package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class openClimb extends CommandBase{
    private final ClimbSubsystem climbSub;
    private final ElevatorSubsystem elevatorSub;

    public openClimb(ClimbSubsystem climbSub, ElevatorSubsystem elevatorSub) {
        this.climbSub = climbSub;
        this.elevatorSub = elevatorSub;

        addRequirements(climbSub, elevatorSub);
    }

    @Override
    public void initialize(){
        elevatorSub.setPosition(12).withTimeout(3);
    }

    @Override
    public void execute(){
        climbSub.openClimb();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
