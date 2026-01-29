package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

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

  // configs
  public static final double gearing = 0.5; // TODO: tune

  public final class ShooterSimConstants {
    // phisics
    public static final double JKG = 0.25; // TODO: tune

    // sims
    public static final DCMotor motorModel = DCMotor.getKrakenX60(2);
    public static final TalonFXSimState simMotor = _masterMotor.getSimState();
    public static final DCMotorSim sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, JKG, gearing), motorModel);
  }
}
