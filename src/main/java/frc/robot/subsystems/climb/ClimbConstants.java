package frc.robot.subsystems.climb;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class ClimbConstants {
  // ids
  public static final int MOTOR_ID = 40;

  // devices
  public static final TalonFX _motor = new TalonFX(MOTOR_ID);

  // signals
  public static final StatusSignal<Voltage> voltageSignal = _motor.getMotorVoltage();
  public static final StatusSignal<Current> statorCurrentSignal = _motor.getStatorCurrent();
  public static final StatusSignal<Current> supplyCurrentSignal = _motor.getSupplyCurrent();
  public static final StatusSignal<Angle> positionSignal = _motor.getPosition();
  public static final StatusSignal<AngularVelocity> velocitySignal = _motor.getVelocity();
  public static final StatusSignal<AngularAcceleration> AccelerationSignal = _motor.getAcceleration();

  // requests
  public static final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);
  public static final PositionVoltage positionRequest = new PositionVoltage(0);
  public static final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  // configs
  public static final double STATOR_CURRENT = 100; // TODO: TUNE
  public static final double SUPPLY_CURRENT = 50; // TODO: TUNE
  public static final double GEAR_RATIO = 100;
  public static final double FORWARD_LIMIT = 10;
  public static final double REVERSE_LIMIT = 0;
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

    //SoftwareLimitSwitch
    realConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    realConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_LIMIT;
    realConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    realConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_LIMIT;



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

        //SoftwareLimitSwitch
      simConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      simConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_LIMIT;
      simConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      simConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_LIMIT;

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

      //sim config
      public static double WEIGHT = 50; // TODO: find
      public static double RADIUS = 50; // TODO: find
      public static double MAXHEIGHT = 50; // TODO: find
      public static boolean DOGRAVITY = true; // TODO: decide

      static public DCMotor motorModel = DCMotor.getFalcon500(2);
      public static ElevatorSim elevatorSim = new ElevatorSim(motorModel,
          GEAR_RATIO,
          WEIGHT,
          RADIUS,
          0,
          MAXHEIGHT,
          DOGRAVITY,
          0);
    }

    
  // mech2d
  public static final Mechanism2d mech2d = new Mechanism2d(3, 3);
  private static final MechanismRoot2d root = mech2d.getRoot("root", 1.5, 0);
  public static final MechanismLigament2d climb2d =
      root.append(new MechanismLigament2d("climb", 0, 90));
  }
