package frc.robot.subsystems.climb;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ClimbConstants {
    // ids
    public static final int id = 0; // TODO: find Id

    // devices
    public static final TalonFX _motor = new TalonFX(id);

    // signals
    public static final StatusSignal<Voltage> voltageSignal = _motor.getMotorVoltage();
    public static final StatusSignal<Current> statorCurrentSignal = _motor.getStatorCurrent();
    public static final StatusSignal<Current> supplyCurrentSignal = _motor.getSupplyCurrent();
    public static final StatusSignal<Angle> postiton = _motor.getPosition();
    public static final StatusSignal<AngularVelocity> velocitySignal = _motor.getVelocity();
    public static final StatusSignal<AngularAcceleration> AccelerationSignal = _motor.getAcceleration();

    // requests
    public static final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);
    public static final PositionVoltage positionRequest = new PositionVoltage(0);
    public static final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    // configs
    public static final double STATOR_CURRENT = 100; // TODO: TUNE
    public static final double SUPPLY_CURRENT = 50; // TODO: TUNE
    public static final double GEAR_RATIO = 0; // TODO: TUNE
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake; // TODO: TUNE
    public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive; // TODO: TUNE

    // config declaration
    public static final TalonFXConfiguration realConfiguration = new TalonFXConfiguration();

    static {

    // peaks:
    realConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    realConfiguration.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT;
    realConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    realConfiguration.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT;

    // settings
    realConfiguration.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    realConfiguration.MotorOutput.NeutralMode = NEUTRAL_MODE;
    realConfiguration.MotorOutput.Inverted = INVERTED;

    // PID
    realConfiguration.Slot0.kP = 8; // TODO: tune
    realConfiguration.Slot0.kI = 0.0;
    realConfiguration.Slot0.kD = 1;

    // feedforward
    realConfiguration.Slot0.kS = 0; // TODO: tune
    realConfiguration.Slot0.kV = 0;
    realConfiguration.Slot0.kA = 0;
  }
  public class HoodSimConstants {
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

      // feedforward
      simConfiguration.Slot0.kS = 0; // TODO: tune
      simConfiguration.Slot0.kV = 0;
      simConfiguration.Slot0.kA = 0;
    }
    public class ClimbSimConstants {
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

      // feedforward
      simConfiguration.Slot0.kS = 0; // TODO: tune
      simConfiguration.Slot0.kV = 0;
      simConfiguration.Slot0.kA = 0;
    }
    public static final TalonFXSimState simMotor = _motor.getSimState();

     public static final  armSim =
}
