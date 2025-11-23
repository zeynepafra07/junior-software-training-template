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
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SparkFlexSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.mechanism2d.Mechanism2d;
import edu.wpi.first.wpilibj.mechanism2d.MechanismLigament2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.math.util.Units;

public class ElevatorSubsystem extends SubsystemBase {
    public SparkFlex masterMotor, slaveMotor;
    public SparkFlexConfig masterConfig, slaveConfig;
    private final ArmFeedforward feedForward;
    private final RelativeEncoder encoder;

    private final SparkClosedLoopController closedLoopController;
    private final ClosedLoopConfig closedLoopConfig;
    private final double gearratio = 24;
    private final double drumRadius = 0.5; //change
    private final DCMotor gearbox = DCMotor.getNEO(2);

    //simulation components
    private final ElevatorSim elevatorSim =
      new ElevatorSim(
          gearbox,
          gearratio,
          4,
          aa,
          Units.inchesToMeters(Constants.Climb.Levels.baseHeight),
          Units.inchesToMeters(Constants.Climb.Levels.maxHeight),
          true,
          0, 0.01, 0.0);
    private final SparkFlexSim motorSim = new SparkFlexSim(masterMotor, gearbox);

    private final Mechanism2d mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 10, 0);
    private final MechanismLigament2d elevatorMech2d = mech2dRoot.append(MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90));

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

        SmartDashboard.putData("Elevator Mechanism", mech2d);
    }

    public void simulationPeriodic(){
        motorSim.setInputVoltage(motorSim.getAppliedOutput()*RobotController.getBatteryVoltage());

        elevatorSim.update(0.02);

        motorSim.setPosition(elevatorSim.getPositionRotations());

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
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
        masterConfig.encoder.setPositionConversionFactor((Math.PI * drumRadius*2)/gearratio);

        slaveConfig.idleMode(IdleMode.kBrake).follow(masterMotor).voltageCompensation(12.0).smartCurrentLimit(45);

        masterMotor.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        slaveMotor.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Actual Position", encoder.getPosition());
        SmartDashboard.putNumber("Elevator Target Position", closedLoopController.getReferenceValue());
        SmartDashboard.putNumber("Elevator Voltage", masterMotor.getAppliedOutput());

        elevatorMech2d.setLength(encoder.getPosition());
    }

    public Command reachPosition(double position){
        return runOnce(() -> setPosition(position)).until(() -> isAtPosition(position));
    }

    @Override
    public void close() {
        encoder.close();
        masterMotor.close();
        slaveMotor.close();
        m_mech2d.close();
    }
}
