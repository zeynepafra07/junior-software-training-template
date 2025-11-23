package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;

import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
    public SparkMax shooterMotor;
    public SparkMaxConfig shooterConfig;

    public ShooterSubsystem(){
        shooterMotor = new SparkMax(Shooter.motorID, SparkMax.MotorType.kBrushless);
        shooterConfig = new SparkMaxConfig();
        configureMotor();
    }

    public void setVoltage(double voltage){
        double finVoltage = MathUtil.clamp(voltage, Shooter.peakReverseVoltage, Shooter.peakForwardVoltage);

        shooterMotor.setVoltage(finVoltage);
    }

    public void feed(){
        setVoltage(Shooter.feedVoltage);
    }

    public void advance(){
        setVoltage(Shooter.advanceVoltage);
    }

    public void shootL1(){
        setVoltage(Shooter.scoreL1Voltage);
    }

    public void shootL23(){
        setVoltage(Shooter.scoreL2L3Voltage);
    }

    public void shootL4(){
        setVoltage(Shooter.scoreL4Voltage);
    }

    public void algaeIntake(){
        shooterMotor.setVoltage(Shooter.algaeVoltage);
    }

    public void algaeOuttake(){
        setVoltage(Shooter.algaeOutVoltage);
    }

    public void holdAlgae(){
        setVoltage(Shooter.algaeHoldVoltage);
    }

    public void shootAlgaeBarge(){
        setVoltage(Shooter.algaeBargeVoltage);
    }
 
    public void stop(){
        shooterMotor.stopMotor();
    }

    public void configureMotor(){
        shooterConfig.idleMode(IdleMode.kCoast).voltageCompensation(12.0).smartCurrentLimit(40);
        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Voltage", shooterMotor.getAppliedOutput());
    }
}