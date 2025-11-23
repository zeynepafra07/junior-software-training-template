package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command{
    private final IntakeSubsystem intakeSub;
    private Timer timer;
    private boolean timerOn, finished;
    private double holdTime = 1.5;

    public Intake(IntakeSubsystem intakeSub) {
        this.intakeSub = intakeSub;
        timer = new Timer();

        addRequirements(intakeSub);
    }

    @Override
    public void initialize(){
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
