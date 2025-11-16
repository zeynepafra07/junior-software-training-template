package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.Climb;


public class ClimbSubsystem extends SubsystemBase {
    public SparkMax climbMotor;
    public SparkMaxConfig climbConfig;

    public ClimbSubsystem(){
        climbMotor = new SparkMax(Constants.Climb.Climber.motorID, MotorType.kBrushless);
        climbConfig = new SparkMaxConfig();
        configureMotor();
    }

    public void setVoltage(double voltage){
        double finVoltage = MathUtil.clamp(voltage, Constants.Climb.Climber.peakReverseVoltage, Constants.Climb.Climber.peakForwardVoltage);

        climbMotor.setVoltage(finVoltage);
    }

    public void openClimbFast(){
        setVoltage(Constants.Climb.Climber.fastDeployVoltage);
    }

    public void openClimbSlow(){
        setVoltage(Constants.Climb.Climber.slowDeployVoltage);
    }

    public void closeClimbFast(){
        setVoltage(Constants.Climb.Climber.fastRetractVoltage);
    }

    public void closeClimbSlow(){
        setVoltage(Constants.Climb.Climber.slowRetractVoltage);
    }

    public void closeClimb(){
        setVoltage(Constants.Climb.Climber.engageRetractVoltage);
    }

    public void holdClimb(){
        setVoltage(Constants.Climb.Climber.holdVoltage);
    }

    public void configureMotor(){
        climbConfig.idleMode(IdleMode.kBrake).voltageCompensation(12.0).smartCurrentLimit(40);
        climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        //later
    }

}
