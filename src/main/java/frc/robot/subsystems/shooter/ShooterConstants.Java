package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class ShooterConstants {
  // ids
  public static final int MASTER_MOTOR_ID = 30;
  public static final int FOLLOWER_MOTOR_ID = 31;

  // devices
  public static final TalonFX _masterMotor = new TalonFX(MASTER_MOTOR_ID, Constants.CAN_BUS_NAME);
  public static final TalonFX _followerMotor =
      new TalonFX(FOLLOWER_MOTOR_ID, Constants.CAN_BUS_NAME);

  // signals
  public static final StatusSignal<Voltage> currentVoltage = _masterMotor.getMotorVoltage();
  public static final StatusSignal<Current> currentStatorCurrent = _masterMotor.getStatorCurrent();
  public static final StatusSignal<Current> currentSupplyCurrent = _masterMotor.getSupplyCurrent();
  public static final StatusSignal<AngularVelocity> currentVelocity = _masterMotor.getVelocity();
  public static final StatusSignal<AngularAcceleration> currentAcceleration =
      _masterMotor.getAcceleration();

  // request
  public static final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);
  public static final MotionMagicVelocityVoltage velocityRequest =
      new MotionMagicVelocityVoltage(0); // rps
  public static final Follower followerRequest = new Follower(MASTER_MOTOR_ID, true);

  // targets values
  public static final double PASS_RPS = 20;
  public static double targetVelocity = 0;
  public static final DoubleSupplier shotVelocitySupplier = () -> 0; // TODO: find equation

  // configs
  public static final double GEAR_RATIO = 0.5; // TODO: tune
  public static final double PEAK_CURRENT = 80; // TODO: tune
  public static final double SUPPLY_CURRENT = 50; // TODO: tune
  public static final double TOLERANCE = 1; // TODO: tune

  public static final TalonFXConfiguration realConfigs = new TalonFXConfiguration();

  static {

    // peaks:
    realConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    realConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT;

    realConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    realConfigs.CurrentLimits.StatorCurrentLimit = PEAK_CURRENT;

    // settings
    realConfigs.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    // TODO: tune motor modes
    realConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    realConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // PID
    // TODO: tune PID
    realConfigs.Slot0.kP = 10;
    realConfigs.Slot0.kI = 0;
    realConfigs.Slot0.kD = 0.03;

    // feedforward
    // TODO: tune FF
    realConfigs.Slot0.kV = 0.16;
    realConfigs.Slot0.kA = 0.02;
    realConfigs.Slot0.kS = 0.17;

    // MM
    // TODO: tune MM
    realConfigs.MotionMagic.MotionMagicCruiseVelocity = 1.5;
    realConfigs.MotionMagic.MotionMagicAcceleration = 0;
  }

  public final class ShooterSimConstants {
    // phisics
    public static final double JKG = 0.5; // TODO: tune

    // sims
    public static final DCMotor motorModel = DCMotor.getKrakenX60(2);
    public static final TalonFXSimState simMotor = _masterMotor.getSimState();
    public static final DCMotorSim sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, JKG, GEAR_RATIO), motorModel);
  }
}
