package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class getL3Algae extends SequentialCommandGroup{
    private final ShooterSubsystem shooterSub;
    private final ElevatorSubsystem elevatorSub;

    publicc getL3Algae(ShooterSubsystem shooterSub, ElevatorSubsystem elevatorSub) {
        this.shooterSub = shooterSub;
        this.elevatorSub = elevatorSub;

        addRequirements(shooterSub, elevatorSub);

        addCommands(
            elevatorSub.setPosition(Constants.Climb.Levels.L3_ALGAE).withTimeout(3),
            new InstantCommand(() -> shooterSub.algaeIntake(), shooterSub),
            new WaitCommand(2),
            new InstantCommand(() -> shooterSub.stop(), shooterSub),
            elevatorSub.setPosition(Constants.Climb.Levels.HOLD).withTimeout(3)
        );
    }
}
