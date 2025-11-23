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
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
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
    public PIDController pidController;
    private final ClosedLoopConfig closedLoopConfig;
    private final double gearratio = 24;
    private final double drumRadius = 0.02; 
    public double setPoint;
    private final DCMotor gearbox = DCMotor.getNEO(2);

    //simulation components
    private final ElevatorSim elevatorSim;
    private final SparkFlexSim motorSim;

    private final Mechanism2d mech2d;
    private final MechanismRoot2d mech2dRoot;
    private final MechanismLigament2d elevatorMech2d;

    public ElevatorSubsystem(){
        masterMotor = new SparkFlex(Constants.Elevator.masterID, SparkFlex.MotorType.kBrushless);
        slaveMotor = new SparkFlex(Constants.Elevator.slaveID, SparkFlex.MotorType.kBrushless);
        masterConfig = new SparkFlexConfig();
        slaveConfig = new SparkFlexConfig();
        encoder = masterMotor.getEncoder();
        pidController = new PIDController(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);

        elevatorSim =new ElevatorSim(
          gearbox,
          gearratio,
          4,
          drumRadius,
          Units.inchesToMeters(Constants.Climb.Levels.baseHeight),
          Units.inchesToMeters(Constants.Climb.Levels.maxHeight),
          true,
          0, 0.01, 0.0);

        
        motorSim = new SparkFlexSim(null, gearbox);

        mech2d = new Mechanism2d(20, 50);
        mech2dRoot = mech2d.getRoot("Elevator Root", 10, 0);
        elevatorMech2d = mech2dRoot.append(new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90));

        closedLoopController = masterMotor.getClosedLoopController();
        closedLoopConfig = new ClosedLoopConfig();

        feedForward = new ArmFeedforward(Constants.Elevator.kS, Constants.Elevator.kG, Constants.Elevator.kV, Constants.Elevator.kA);
    
        configureMotors();

        SmartDashboard.putData("Elevator Mechanism", mech2d);
    }

    public void simulationPeriodic(){
        motorSim.setBusVoltage(motorSim.getAppliedOutput()*RobotController.getBatteryVoltage());

        elevatorSim.update(0.02);

        motorSim.setPosition((elevatorSim.getPositionMeters()/(Math.PI*drumRadius*2))*gearratio);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }

    public void setPosition(double setPoint){
        this.setPoint = setPoint;
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
        closedLoopConfig.p(pidController.getP())
                        .i(pidController.getI())
                        .d(pidController.getD());

        masterConfig.idleMode(IdleMode.kBrake).voltageCompensation(12.0).smartCurrentLimit(45).closedLoop.apply(closedLoopConfig);
        masterConfig.encoder.positionConversionFactor((2 * Math.PI * drumRadius) / gearratio);

        slaveConfig.idleMode(IdleMode.kBrake).follow(masterMotor).voltageCompensation(12.0).smartCurrentLimit(45);

        masterMotor.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        slaveMotor.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Actual Position", encoder.getPosition());
        SmartDashboard.putNumber("Elevator Target Position", setPoint);
        SmartDashboard.putNumber("Elevator Voltage", masterMotor.getAppliedOutput());

        elevatorMech2d.setLength(encoder.getPosition());
    }

    public Command reachPosition(double position){
        return runOnce(() -> setPosition(position)).until(() -> isAtPosition(position));
    }

    public Command simulationCommand(){
        return run(() -> simulationPeriodic());
    }
}