package frc.robot.subsystems.intake.intakeRoller;

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

public class IntakeRollerConstants {

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
  public static final double STATOR_CURRENT = 100;
  public static final double SUPPLY_CURRENT = 50;
  public static final double GEAR_RATIO = 3;
  public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
  public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

  //  Real config declaration
  public static final TalonFXConfiguration realConfigs = new TalonFXConfiguration();

  static {

    // peaks:
    realConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    realConfigs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT;
    realConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    realConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT;

    // settings
    realConfigs.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    realConfigs.MotorOutput.NeutralMode = NEUTRAL_MODE;
    realConfigs.MotorOutput.Inverted = INVERTED;
  }

  public final class intakeRollerSimConstants {

    //  sim config declaration
    public static final TalonFXConfiguration simConfigs = new TalonFXConfiguration();

    static {

      // peaks:
      simConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
      simConfigs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT;
      simConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
      simConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT;

      // settings
      simConfigs.Feedback.SensorToMechanismRatio = GEAR_RATIO;
      simConfigs.MotorOutput.NeutralMode = NEUTRAL_MODE;
      simConfigs.MotorOutput.Inverted = INVERTED;
    }

    // phisics
    public static final double JKG = 0.02; // TODO: tune

    // sims
    public static final DCMotor motorModel = DCMotor.getFalcon500(1);
    public static final TalonFXSimState simMotor = _motor.getSimState();
    public static final FlywheelSim sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(motorModel, JKG, GEAR_RATIO), motorModel);
  }
}
