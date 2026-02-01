package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.ShooterSimConstants.*;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.MathUtil;

public class ShooterSimIO implements ShooterIO {

  public ShooterSimIO() {
    _masterMotor.getConfigurator().apply(simConfigs); // ^ right now this is real configs
    _followerMotor.setControl(followerRequest);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // gets deta
    simMotor.setSupplyVoltage(12);
    sim.setInputVoltage(MathUtil.clamp(simMotor.getMotorVoltage(), -12.0, 12.0));
    sim.update(0.02);
    simMotor.setRotorVelocity(sim.getAngularVelocity().times(GEAR_RATIO));

    StatusSignal.refreshAll(
        currentVoltage,
        currentSupplyCurrent,
        currentStatorCurrent,
        currentVelocity,
        currentAcceleration);
    inputs.voltage = currentVoltage.getValueAsDouble();
    inputs.statorCurrent = currentStatorCurrent.getValueAsDouble();
    inputs.supplyCurrent = sim.getCurrentDrawAmps();
    inputs.velocity = currentVelocity.getValueAsDouble();
    inputs.acceleration = currentAcceleration.getValueAsDouble();
    inputs.wantedVelocity = targetVelocity;
  }
}
