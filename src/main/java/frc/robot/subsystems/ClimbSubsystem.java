package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;

import frc.robot.Constants.Climb;


public class ClimbSubsystem extends SubsystemBase {
    public SparkMax climbMotor;
    public SparkMaxConfig climbConfig;
    public int climbMode = 1;

    public ClimbSubsystem(){
        climbMotor = new SparkMax(Climb.Climber.motorID, SparkMax.MotorType.kBrushless);
        climbConfig = new SparkMaxConfig();
        configureMotor();
    }

    public void setVoltage(double voltage){
        double finVoltage = MathUtil.clamp(voltage, Climb.Climber.peakReverseVoltage, Climb.Climber.peakForwardVoltage);

        climbMotor.setVoltage(finVoltage);
    }

    public void openClimbFast(){
        setVoltage(Climb.Climber.fastDeployVoltage);
    }

    public void openClimbSlow(){
        setVoltage(Climb.Climber.slowDeployVoltage);
    }

    public void openClimb(){
        if(climbMode == 1){
            openClimbFast();
        } else if (climbMode == 0){
            openClimbSlow();
        }
    }

    public void closeClimbFast(){
        setVoltage(Climb.Climber.fastRetractVoltage);
    }

    public void closeClimbSlow(){
        setVoltage(Climb.Climber.slowRetractVoltage);
    }

    public void closeClimb(){
        if(climbMode == 1){
            closeClimbFast();
        } else if (climbMode == 0){
            closeClimbSlow();
        }
    }

    public void engageClimb(){
        setVoltage(Climb.Climber.engageRetractVoltage);
    }

    public void holdClimb(){
        setVoltage(Climb.Climber.holdVoltage);
    }

    public void configureMotor(){
        climbConfig.idleMode(IdleMode.kBrake).voltageCompensation(12.0).smartCurrentLimit(40);
        climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Motor Voltage", climbMotor.getAppliedOutput());
    }
}