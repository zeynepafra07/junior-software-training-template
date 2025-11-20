package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class L1Shooting extends SequentialCommandGroup {
    private final ShooterSubsystem shooterSub;
    private final ElevatorSubsystem elevatorSub;

    public L1Shooting(ShooterSubsystem shooterSub, ElevatorSubsystem elevatorSub) {
        this.shooterSub = shooterSub;
        this.elevatorSub = elevatorSub;

        addRequirements(shooterSub, elevatorSub);

        addCommands(
            elevatorSub.setPosition(Constants.Climb.Levels.L4_SCORE).withTimeout(3),
            new InstantCommand(() -> shooterSub.shootL4(), shooterSub),
            new WaitCommand(1.5),
            new InstantCommand(() -> shooterSub.stop(), shooterSub),
            elevatorSub.setPosition(Constants.Climb.Levels.SAFE).withTimeout(3);
        );
    }
}
