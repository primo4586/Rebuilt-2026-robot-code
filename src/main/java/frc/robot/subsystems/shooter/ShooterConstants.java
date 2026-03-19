package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.util.interpolation.InterpolationMap;
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
  public static final StatusSignal<Voltage> voltageSignal = _masterMotor.getMotorVoltage();
  public static final StatusSignal<Current> statorCurrentSignal = _masterMotor.getStatorCurrent();
  public static final StatusSignal<Current> supplyCurrentSignal = _masterMotor.getSupplyCurrent();
  public static final StatusSignal<AngularVelocity> velocitySignal = _masterMotor.getVelocity();
  public static final StatusSignal<Angle> positionSignal = _masterMotor.getPosition();
  public static final StatusSignal<AngularAcceleration> AccelerationSignal =
      _masterMotor.getAcceleration();
  // request
  public static final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);
  public static final MotionMagicVelocityVoltage velocityRequest =
      new MotionMagicVelocityVoltage(0).withEnableFOC(true); // rps
  public static final Follower followerRequest = new Follower(MASTER_MOTOR_ID, MotorAlignmentValue.valueOf(1));

  // targets values
  public static final double PASS_RPS = 20;
  public static final double REST_VELOCITY = 4;
  public static final double SHOOT_RPS = 20; // TODO: tune
  public static double targetVelocity = 0;
  public static final DoubleSupplier shotVelocitySupplier = () -> 0; // TODO: find equation

  // Interpolation Map
  public static final InterpolationMap SHOOTER_INTERPOLATION_MAP =
      new InterpolationMap().put(2.577, 100).put(2.124, 100).put(3.2, 100); // TODO: Find points

  // configs
  public static final double GEAR_RATIO = 28/18;
  public static final double PEAK_CURRENT = 200; // TODO: tune
  public static final double SUPPLY_CURRENT = 200; // TODO: tune
  public static final double TOLERANCE = 0.435;

  public static final TalonFXConfiguration realConfigs = new TalonFXConfiguration();

  static {

    // peaks:
    realConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
    realConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT;

    realConfigs.CurrentLimits.StatorCurrentLimitEnable = false;
    realConfigs.CurrentLimits.StatorCurrentLimit = PEAK_CURRENT;

    // settings
    realConfigs.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    // TODO: tune motor modes
    realConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    realConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // PID
    realConfigs.Slot0.kP = 0.00012279;
    realConfigs.Slot0.kI = 0;
    realConfigs.Slot0.kD = 0; //TODO: maybe add D 

    // feedforward 
    realConfigs.Slot0.kV = 0.12219;
    realConfigs.Slot0.kA = 0.0087164;
    realConfigs.Slot0.kS = 0.28562;

    // MM
    // TODO: tune MM
    realConfigs.MotionMagic.MotionMagicAcceleration = 1000;
  }

  public final class ShooterSimConstants {
    public static final TalonFXConfiguration simConfigs = new TalonFXConfiguration();

    static {

      // peaks:
      simConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
      simConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT;

      simConfigs.CurrentLimits.StatorCurrentLimitEnable = false;
      simConfigs.CurrentLimits.StatorCurrentLimit = PEAK_CURRENT;

      // settings
      simConfigs.Feedback.SensorToMechanismRatio = GEAR_RATIO;
      // TODO: tune motor modes
      simConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      simConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      // PID
      // TODO: tune PID
      simConfigs.Slot0.kP = 10;
      simConfigs.Slot0.kI = 0;
      simConfigs.Slot0.kD = 0.03;

      // feedforward
      // TODO: tune FF
      simConfigs.Slot0.kV = 0.16;
      simConfigs.Slot0.kA = 0.02;
      simConfigs.Slot0.kS = 0.17;

      // MM
      // TODO: tune MM;
      simConfigs.MotionMagic.MotionMagicAcceleration = 3;
    }

    // physics
    public static final double JKG = 0.05; // TODO: tune

    // sims
    public static final DCMotor motorModel = DCMotor.getKrakenX60(2);
    public static final TalonFXSimState simMotor = _masterMotor.getSimState();
    public static final FlywheelSim sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(motorModel, JKG, GEAR_RATIO), motorModel);
  }
}
