package frc.robot.subsystems.intake.intakeArm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants;

public class IntakeArmConstants {

  // ids
  public static final int MOTOR_ID = 21;

  // devices
  public static final TalonFX _motor = new TalonFX(MOTOR_ID, Constants.CAN_BUS_NAME);

  // signals
  public static final StatusSignal<Voltage> currentVoltage = _motor.getMotorVoltage();
  public static final StatusSignal<Current> currentStatorCurrent = _motor.getStatorCurrent();
  public static final StatusSignal<Current> currentSupplyCurrent = _motor.getSupplyCurrent();
  public static final StatusSignal<AngularVelocity> currentVelocity = _motor.getVelocity();
  public static final StatusSignal<Angle> currentPosition = _motor.getPosition();

  // request
  public static final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);
  public static final PositionVoltage positionRequest = new PositionVoltage(0);

  // configs
  public static final double STATOR_CURRENT = 100;
  public static final double SUPPLY_CURRENT = 50;
  public static final double GEAR_RATIO = 5;
  public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
  public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
  public static final double gearing = 0.5; // TODO: tune

  // config declaration

  public static final TalonFXConfiguration realConfiguration = new TalonFXConfiguration();

  static {

    // peaks:
    realConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    realConfiguration.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT;
    realConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    realConfiguration.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT;

    // settings
    realConfiguration.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    realConfiguration.MotorOutput.NeutralMode = NEUTRAL_MODE;
    realConfiguration.MotorOutput.Inverted = INVERTED;

    // PID
    realConfiguration.Slot0.kP = 8; // TODO: tune
    realConfiguration.Slot0.kI = 0.0;
    realConfiguration.Slot0.kD = 1;
  }

  public static final TalonFXConfiguration simConfiguration = new TalonFXConfiguration();

  static {

    // peaks:
    simConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    simConfiguration.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT;
    simConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    simConfiguration.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT;

    // settings
    simConfiguration.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    simConfiguration.MotorOutput.NeutralMode = NEUTRAL_MODE;
    simConfiguration.MotorOutput.Inverted = INVERTED;

    // PID
    simConfiguration.Slot0.kP = 10; // TODO: tune
    simConfiguration.Slot0.kI = 0.0;
    simConfiguration.Slot0.kD = 1;
  }

  public static final double startingAngleRads = 0;

  public class IntakeArmSimConstants {
    public static final TalonFXSimState simMotor = _motor.getSimState();

    // sim constants
    public static final DCMotor GEARBOX = DCMotor.getFalcon500(1);
    public static final double jKgMetersSquared = 0.14;
    public static final double armLengthMeters = 0.56;
    // public static final double minAngleRads = Math.PI / 2;
    // public static final double maxAngleRads = 1.25 * Math.PI;
    public static final double minAngleRads = Math.toRadians(90) * -100;
    public static final double maxAngleRads = Math.toRadians(180) * 100;

    public static final SingleJointedArmSim armSim =
        new SingleJointedArmSim(
            GEARBOX,
            GEAR_RATIO,
            jKgMetersSquared,
            armLengthMeters,
            minAngleRads,
            maxAngleRads,
            false,
            startingAngleRads);
  }

  // Mech2d
  public static final Mechanism2d mech2d = new Mechanism2d(150, 150);
  public static final MechanismRoot2d root = mech2d.getRoot("root", 75, 75);
  public static final MechanismLigament2d arm =
      root.append(new MechanismLigament2d("arm", 65, startingAngleRads));
}
