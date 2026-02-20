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
    statusCode =
        StatusSignal.refreshAll(
            voltageSignal,
            supplyCurrentSignal,
            statorCurrentSignal,
            velocitySignal,
            AccelerationSignal);

    inputs.voltage = voltageSignal.getValueAsDouble();
    inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
    inputs.velocity = velocitySignal.getValueAsDouble();
    inputs.acceleration = AccelerationSignal.getValueAsDouble();
  }
}
