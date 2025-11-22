package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ResetEncoder extends SequentialCommandGroup{
    private final ElevatorSubsystem elevatorSub;

    public ResetEncoder(ElevatorSubsystem elevatorSub) {
        this.elevatorSub = elevatorSub;
        addRequirements(elevatorSub);

        addCommands(
            new Command() -> elevatorSub.moveManual(-5).withTimeout(2),
            new InstantCommand(() -> elevatorSub.resetEncoder(), elevatorSub)
        );
    }
    
}
