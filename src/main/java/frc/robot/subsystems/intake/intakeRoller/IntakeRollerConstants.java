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
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeRollerConstants {

  // ids
  public static final int MOTOR_ID = 20;

  // devices
  public static final TalonFX _motor = new TalonFX(MOTOR_ID, Constants.CAN_BUS_NAME);

  // signals
  public static final StatusSignal<Voltage> currentVoltage = _motor.getMotorVoltage();
  public static final StatusSignal<Current> currentStatorCurrent = _motor.getStatorCurrent();
  public static final StatusSignal<Current> currentSupplyCurrent = _motor.getSupplyCurrent();
  public static final StatusSignal<AngularVelocity> currentVelocity = _motor.getVelocity();

  // request
  public static final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);

  // configs
  public static final double STATOR_CURRENT = 100;
  public static final double SUPPLY_CURRENT = 50;
  public static final double GEAR_RATIO = 5;
  public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
  public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;
  public static final double gearing = 0.5; // TODO: tune

  //  Real config declaration
  public static final TalonFXConfiguration configuration = new TalonFXConfiguration();

  static {

    // peaks:
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT;
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT;

    // settings
    configuration.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    configuration.MotorOutput.NeutralMode = NEUTRAL_MODE;
    configuration.MotorOutput.Inverted = INVERTED;
  }

  public final class intakeRollerSimConstants {
    // phisics
    public static final double JKG = 0.25; // TODO: tune

    // sims
    public static final DCMotor motorModel = DCMotor.getFalcon500(1);
    public static final TalonFXSimState simMotor = _motor.getSimState();
    public static final DCMotorSim sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, JKG, gearing), motorModel);
  }
}
