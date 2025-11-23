package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ResetEncoder extends SequentialCommandGroup{
    private final ElevatorSubsystem elevatorSub;

    public ResetEncoder(ElevatorSubsystem elevatorSub) {
        this.elevatorSub = elevatorSub;
        addRequirements(elevatorSub);

        addCommands(
            new InstantCommand(() -> elevatorSub.moveManual(-5)).withTimeout(2),
            new InstantCommand(() -> elevatorSub.reset(), elevatorSub)
        );
    }
    
}
