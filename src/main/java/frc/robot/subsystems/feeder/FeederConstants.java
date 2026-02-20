package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class FeederConstants {

  // ids
  public static final int MOTOR_ID = 20;

  // devices
  public static final TalonFX _motor = new TalonFX(MOTOR_ID, Constants.CAN_BUS_NAME);

  // signals
  public static final StatusSignal<Voltage> voltageSignal = _motor.getMotorVoltage();
  public static final StatusSignal<Current> statorCurrentSignal = _motor.getStatorCurrent();
  public static final StatusSignal<Current> supplyCurrentSignal = _motor.getSupplyCurrent();
  public static final StatusSignal<AngularVelocity> velocitySignal = _motor.getVelocity();

  // request
  public static final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);

  // configs
  public static final double STATOR_CURRENT = 60; // TODO: tune current
  public static final double SUPPLY_CURRENT = 30;
  public static final double VOLTAGE_LIMIT = 0;
  public static final double GEAR_RATIO = 3; // TODO: set gear ratio
  public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
  public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

  // Real config declaration
  public static final TalonFXConfiguration realConfigs = new TalonFXConfiguration();

  static {

    // peaks:
    realConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    realConfigs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT;
    realConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    realConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT;
    realConfigs.Voltage.PeakForwardVoltage = VOLTAGE_LIMIT;
    realConfigs.Voltage.PeakReverseVoltage = VOLTAGE_LIMIT;

    // settings
    realConfigs.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    realConfigs.MotorOutput.NeutralMode = NEUTRAL_MODE;
    realConfigs.MotorOutput.Inverted = INVERTED;
  }

  // Default Values
  public static final double FEED_VOLTAGE = 6; // TODO: set voltage
  public static final double FEED_CURRENT = 30;

  public final class FeederSimConstants {

    // sim config declaration
    public static final TalonFXConfiguration simConfigs = new TalonFXConfiguration();

    static {

      // peaks:
      simConfigs.CurrentLimits.StatorCurrentLimitEnable = false;
      simConfigs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT;
      simConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
      simConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT;
      simConfigs.Voltage.PeakForwardVoltage = VOLTAGE_LIMIT;
      simConfigs.Voltage.PeakReverseVoltage = VOLTAGE_LIMIT;
      // settings
      simConfigs.Feedback.SensorToMechanismRatio = GEAR_RATIO;
      simConfigs.MotorOutput.NeutralMode = NEUTRAL_MODE;
      simConfigs.MotorOutput.Inverted = INVERTED;
    }

    // phisics
    public static final double JKG = 5; // TODO: tune

    // sims
    public static final DCMotor motorModel = DCMotor.getFalcon500(2);
    public static final TalonFXSimState simMotor = _motor.getSimState();
    public static final FlywheelSim sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(motorModel, JKG, GEAR_RATIO), motorModel);
  }
}
