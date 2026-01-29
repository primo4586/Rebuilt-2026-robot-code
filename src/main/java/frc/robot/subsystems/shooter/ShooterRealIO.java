package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;

public class ShooterRealIO implements ShooterIO {

public ShooterRealIO() {
    _followerMotor.setControl(followerRequest);
}
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    StatusSignal.refreshAll(
        currentVoltage,
        currentSupplyCurrent,
        currentStatorCurrent,
        currentVelocity,
        currentAcceleration);
    inputs.voltage = currentVoltage.getValueAsDouble();
    inputs.statorCurrent = currentStatorCurrent.getValueAsDouble();
    inputs.supplyCurrent = currentSupplyCurrent.getValueAsDouble();
    inputs.velocity = currentVelocity.getValueAsDouble();
    inputs.acceleration = currentAcceleration.getValueAsDouble();
  }
}
