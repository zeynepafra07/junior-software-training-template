package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.SparkFlex.MotorType;
import com.revrobotics.spark.encoder.RelativeEncoder;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Climb.Levels;
import edu.wpi.first.math.controller.ArmFeedforward;

public class ElevatorSubsystem extends SubsystemBase {
    public SparkFlex masterMotor, slaveMotor;
    public SparkFlexConfig masterConfig, slaveConfig;
    private final ArmFeedforward feedForward;
    private final RelativeEncoder encoder;

    private final SparkClosedLoopController closedLoopController;
    private final ClosedLoopConfig closedLoopConfig;
    private final double gearratio = 10.71; //Gear ratio is unknown, change later

    public ElevatorSubsystem(){
        masterMotor = new SparkFlex(Constants.Elevator.masterID, MotorType.kBrushless);
        slaveMotor = new SparkFlex(Constants.Elevator.slaveID, MotorType.kBrushless);
        masterConfig = new SparkFlexConfig();
        slaveConfig = new SparkFlexConfig();
        encoder = masterMotor.getEncoder();

        closedLoopController = masterMotor.getClosedLoopController();
        closedLoopConfig = new ClosedLoopConfig();

        feedForward = new ArmFeedforward(Constants.Elevator.kS, Constants.Elevator.kG, Constants.Elevator.kV, Constants.Elevator.kA);
    
        configureMotors();
    }

    public void setPosition(double setPoint){
        closedLoopController.setReference(setPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0,feedForward.calculate(setPoint, 0) );

    }

    public boolean isAtPosition(double position){
        double tolerance = 0.1;
        return Math.abs(encoder.getPosition() - position) < tolerance;
    }

    public void moveManual(double voltage){
        masterMotor.setVoltage(voltage);
    }

    public void stop(){
        masterMotor.stopMotor();
    }

    public void reset(){
        encoder.setPosition(12);
    }

    public void configureMotors(){
        closedLoopConfig.pid.kP(Constants.Elevator.kP).kI(Constants.Elevator.kI).kD(Constants.Elevator.kD);

        masterConfig.idleMode(IdleMode.kBrake).voltageCompensation(12.0).smartCurrentLimit(45).closedLoop.apply(closedLoopConfig);
        masterConfig.encoder.setPositionConversionFactor(Math.PI * gearratio);

        slaveConfig.idleMode(IdleMode.kBrake).follow(masterMotor).voltageCompensation(12.0).smartCurrentLimit(45);

        masterMotor.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        slaveMotor.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Actual Position", encoder.getPosition());
        SmartDashboard.putNumber("Elevator Target Position", closedLoopController.getReferenceValue());
        SmartDashboard.putNumber("Elevator Voltage", masterMotor.getAppliedOutput());
    }

    public Command reachPosition(double position){
        return runOnce(() -> setPosition(position)).until(() -> isAtPosition(position));
    }

}
