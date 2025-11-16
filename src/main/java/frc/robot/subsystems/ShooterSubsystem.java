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

import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
    public SparkMax shooterMotor;
    public SparkMaxConfig shooterConfig;

    public ShooterSubsystem(){
        masterMotor = new SparkMax(Shooter.motorID, MotorType.kBrushless);
        shooterConfig = new SparkMaxConfig();
        configureMotor();
    }

    public void setVoltage(double voltage){
        double finVoltage = MathUtil.clamp(voltage, Constants.Shooter.peakReverseVoltage, Constants.Shooter.peakForwardVoltage);

        shooterMotor.setVoltage(finVoltage);
    }

    public void feed(){
        setVoltage(Constants.Shooter.feedVoltage);
    }

    public void advance(){
        setVoltage(Constants.Shooter.advanceVoltage);
    }

    public void shootL1(){
        setVoltage(Constants.Shooter.scoreL1Voltage);
    }

    public void shootL23(){
        setVoltage(Constants.Shooter.scoreL2L3Voltage);
    }

    public void shootL4(){
        setVoltage(Constants.Shooter.scoreL4Voltage);
    }

    public void algaeIntake(){
        shooterMotor.setVoltage(Constants.Shooter.algaeVoltage);
    }

    public void algaeOuttake(){
        setVoltage(Constants.Shooter.algaeOutVoltage);
    }

    public void holdAlgae(){
        setVoltage(Constants.Shooter.algaeHoldVoltage);
    }

    public void shootAlgaeBarge(){
        setVoltage(Constants.Shooter.algaeBargeVoltage);
    }

    public void 
    public void stop(){
        shooterMotor.stopMotor();
    }

    public void configureMotor(){
        shooterConfig.idleMode(IdleMode.kCoast).voltageCompensation(12.0).smartCurrentLimit(40);
        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        //later
    }
}
