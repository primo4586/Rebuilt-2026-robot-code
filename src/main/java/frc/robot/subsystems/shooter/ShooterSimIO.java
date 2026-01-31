package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.ShooterSimConstants.*;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ShooterSimIO implements ShooterIO {

  public ShooterSimIO() {
    _masterMotor.getConfigurator().apply(realConfigs); // ^ right now this is real configs
    _followerMotor.setControl(followerRequest);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // gets deta
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    sim.setInputVoltage(MathUtil.clamp(simMotor.getMotorVoltage(), -12.0, 12.0));
    sim.update(0.02);

    simMotor.setRotorVelocity(sim.getAngularVelocityRadPerSec() / GEAR_RATIO);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

    StatusSignal.refreshAll(
        currentVoltage,
        currentSupplyCurrent,
        currentStatorCurrent,
        currentVelocity,
        currentAcceleration);
    inputs.voltage = currentVoltage.getValueAsDouble();
    inputs.statorCurrent = currentStatorCurrent.getValueAsDouble();
    inputs.supplyCurrent = sim.getCurrentDrawAmps();
    inputs.velocity = currentVelocity.getValueAsDouble() * GEAR_RATIO;
    inputs.acceleration = currentAcceleration.getValueAsDouble();
    inputs.wantedVelocity = targetVelocity;
  }
}
