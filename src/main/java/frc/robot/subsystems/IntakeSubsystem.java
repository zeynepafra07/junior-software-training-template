package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.MathUtil;

import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase {
    public SparkMax intakeMotor;
    public SparkMaxConfig intakeConfig;
    private DigitalInput intakeSensor;

    public IntakeSubsystem(){
        intakeMotor = new SparkMax(Intake.motorID, SparkMax.MotorType.kBrushless);
        intakeConfig = new SparkMaxConfig();
        intakeSensor = new DigitalInput(Intake.sensorID);
        configureMotor();

    }

    public void setVoltage(double voltage){
        double finVoltage = MathUtil.clamp(voltage, Intake.peakReverseVoltage, Intake.peakForwardVoltage);

        intakeMotor.setVoltage(finVoltage);
    }

    public void intake(){
        setVoltage(Intake.intakeVoltage);
    }

    public void outtake(){
        setVoltage(Intake.rejectVoltage);
    }

    public void stop(){
        intakeMotor.stopMotor();
    }

    public boolean hasCoral(){
        return !intakeSensor.get();
    }

    public void configureMotor(){
        intakeConfig.idleMode(IdleMode.kCoast).voltageCompensation(12.0).smartCurrentLimit(30);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Sensor", hasCoral());
        SmartDashboard.putNumber("Intake Voltage", intakeMotor.getAppliedOutput());
    }
}
