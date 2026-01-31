package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

public class ShooterRealIO implements ShooterIO {
  StatusCode statusCode = StatusCode.StatusCodeNotInitialized;

  public ShooterRealIO() {
    // upload configs
    for (int i = 0; i < 5; i++) {
      statusCode = _masterMotor.getConfigurator().apply(realConfigs);
      if (statusCode.isOK()) break;
    }
    if (!statusCode.isOK()) System.out.println("shooter configs failed" + statusCode.toString());

    // follower
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
    inputs.wantedVelocity = targetVelocity;
  }
}
