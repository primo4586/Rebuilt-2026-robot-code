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
import edu.wpi.first.units.measure.AngularAcceleration;
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
  public static final StatusSignal<AngularAcceleration> currentAcceleration =
      _motor.getAcceleration();

  // request
  public static final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);

  // configs
  public static final double PEAK_CURRENT = 0;
  public static final double CURRENT_LIMIT = 0;
  public static final double GEAR_RATIO = 0;
  public static final NeutralModeValue NEUTRAL_MODE = null;
  public static final InvertedValue INVERTED = null;
  public static final double gearing = 0.5; // TODO: tune

   //  Real config declaration
  public static final TalonFXConfiguration realConfiguration = new TalonFXConfiguration();
    
    static {
      
    // peaks:
        realConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        realConfiguration.CurrentLimits.StatorCurrentLimit = PEAK_CURRENT;
        realConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        realConfiguration.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
  
    // settings
        realConfiguration.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        realConfiguration.MotorOutput.NeutralMode = NEUTRAL_MODE;
        realConfiguration.MotorOutput.Inverted = INVERTED;
    }

  public final class intakeRollerSimConstants {
    // phisics
    public static final double JKG = 0.25; // TODO: tune

    // sims
    public static final DCMotor motorModel = DCMotor.getKrakenX60(2);
    public static final TalonFXSimState simMotor = _motor.getSimState();
    public static final DCMotorSim sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, JKG, gearing), motorModel);
  }
}
