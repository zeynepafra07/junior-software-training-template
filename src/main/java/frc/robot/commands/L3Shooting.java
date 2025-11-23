package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class L3Shooting extends SequentialCommandGroup {
    private final ShooterSubsystem shooterSub;
    private final ElevatorSubsystem elevatorSub;

    public L3Shooting(ShooterSubsystem shooterSub, ElevatorSubsystem elevatorSub) {
        this.shooterSub = shooterSub;
        this.elevatorSub = elevatorSub;

        addRequirements(shooterSub, elevatorSub);

        addCommands(
            elevatorSub.reachPosition(Constants.Climb.Levels.L3).withTimeout(1),
            new InstantCommand(() -> shooterSub.feed(), shooterSub),
            new WaitCommand(0.1),
            new InstantCommand(() -> shooterSub.shootL23(), shooterSub),
            new WaitCommand(0.7),
            new InstantCommand(() -> shooterSub.stop(), shooterSub),
            elevatorSub.reachPosition(Constants.Climb.Levels.SAFE).withTimeout(3)
        );
    }
}