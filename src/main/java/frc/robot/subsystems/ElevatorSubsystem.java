package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Climb.Levels;


public class ElevatorSubsystem extends SubsystemBase {
    public SparkFlex masterMotor, slaveMotor;
    public SparkFlexConfig masterConfig, slaveConfig;
    private final SparkClosedLoopController closedLoopController;
    private final ClosedLoopConfig closedLoopConfig;
    public PIDController pidController;
    private final SimpleMotorFeedforward feedforward;
    private final RelativeEncoder encoder;

    public ElevatorSubsystem(){
        masterMotor = new SparkFlex(Constants.Elevator.masterID, MotorType.kBrushless);
        slaveMotor = new SparkFlex(Constants.Elevator.slaveID, MotorType.kBrushless);
        masterConfig = new SparkFlexConfig();
        slaveConfig = new SparkFlexConfig();
        closedLoopController = masterMotor.getClosedLoopController();
        closedLoopConfig = new ClosedLoopConfig();
        encoder = masterMotor.getEncoder();
        feedforward = new SimpleMotorFeedforward(Constants.Elevator.kS, Constants.Elevator.kG, Constants.Elevator.kV, Constants.Elevator.kA);
        pidController = new PIDController(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);

        encoder.setPositionConversionFactor(1.0);
        encoder.setVelocityConversionFactor(1.0);

        configureMotors();
    }

    public double calculateTargetV(double targetPosition){
        currentPosition = encoder.getPosition();
        currentVelocity = encoder.getVelocity();
        if(targetPosition > currentPosition){
            targetVelocity = Math.min(Constants.Elevator.cruiseVelocity, (currentVelocity + calculateTargetA(targetPosition)) * 0.13);
        }
        else{ targetVelocity = Math.max(-Constants.Elevator.cruiseVelocity, (currentVelocity - calculateTargetA(targetPosition)) * 0.13);
        }
        return targetVelocity;
    }
    
    public double calculateTargetA(double targetPosition){
        currentPosition = encoder.getPosition();
        if(targetPosition > currentPosition){
            targetAcceleration = Constants.Elevator.acceleration;
        }
        else{
            targetAcceleration = -Constants.Elevator.acceleration;
        }
        return targetAcceleration;
    }

    public void setPosition(double position){
        double currentPosition = encoder.getPosition();
        double pidOutput = pidController.calculate(currentPosition, position);
        double ffoutpot = feedforward.calculate(calculateTargetV(position), calculateTargetA(position));
    
        masterMotor.setVoltage(pidOutput + ffoutpot);
    }

    public void moveManual(double speed){
        double position = encoder.getPosition();
            
        if(position <= Constants.Climb.Levels.baseHeight && speed < 0){
            masterMotor.stopMotor();
        }

        if(position >= Constants.Climb.Levels.maxHeight && speed > 0){
            masterMotor.stopMotor();
            return;
        }
        
        masterMotor.setVoltage(speed * 2.0);
    }
    

    public void stop(){
        masterMotor.stopMotor();
    }

    public void reset(){
        encoder.setPosition(12.0);
    }

    public void configureMotors(){
        closedLoopConfig.pid(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);
        masterConfig.idleMode(IdleMode.kBrake).voltageCompensation(12.0).smartCurrentLimit(45).closedLoop.apply(closedLoopConfig);
        slaveConfig.idleMode(IdleMode.kBrake).follow(masterMotor).voltageCompensation(12.0).smartCurrentLimit(45);

        masterMotor.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        slaveMotor.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

}
