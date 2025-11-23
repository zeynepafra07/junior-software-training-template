package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
public class Outtake extends Command{
    private final IntakeSubsystem intakeSub;

    public Outtake(IntakeSubsystem intakeSub) {
        this.intakeSub = intakeSub;

        addRequirements(intakeSub);
    }

    @Override
    public void execute(){
        new InstantCommand(() -> intakeSub.outtake(), intakeSub).withTimeout(2);
    }

    @Override
    public boolean isFinished(){
        return false;
    }


}