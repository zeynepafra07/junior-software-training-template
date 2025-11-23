package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Outtake extends CommandBase{
    private final IntakeSubsystem intakeSub;

    public Outtake(IntakeSubsystem intakeSub) {
        this.intakeSub = intakeSub;
        timer = new Timer();

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
