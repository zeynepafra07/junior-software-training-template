package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Intake extends Command{
    private final IntakeSubsystem intakeSub;
    private final Timer timer;
    private final timerOn, finished;
    private final holdTime = 12.0; //unknown change later

    public Intake(IntakeSubsystem intakeSub) {
        this.intakeSub = intakeSub;
        timer = new Timer();

        addRequirements(intakeSub);
    }

    @Override
    public initialize(){
        timer.reset();
        timer.start();
        timerOn = false;
        finished = false;
    }

    @Override
    public void execute(){
        if(!intakeSub.hasCoral()){
            intakeSub.intake();
        }
        else{
            if(!timerOn){
                timer.reset();
                timer.start();
                timerOn = true;
            }
            else if(timer.get() >= holdTime){
                finished = true;
            }
        }
    }

    @Override
    public void end(boolean isFinished){
        intakeSub.stop();
        timer.stop();
        timerOn = false;
        finished = true;
    }

    @Override
    public boolean isFinished(){
        return finished;
    }


}
