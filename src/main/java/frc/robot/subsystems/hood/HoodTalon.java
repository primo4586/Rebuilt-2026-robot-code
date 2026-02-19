package frc.robot.subsystems.hood;

import static frc.robot.subsystems.hood.HoodConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

public class HoodTalon implements HoodIO {

  public HoodTalon() {

    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      _motor.getConfigurator().apply(realConfiguration);
      if (statusCode.isOK()) {
        break;
      }
    }
    if (statusCode.isError()) {
      System.out.println("intkake arm configs failed" + statusCode.toString());
    }
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    StatusSignal.refreshAll(
        voltageSignal, supplyCurrentSignal, statorCurrentSignal, velocitySignal);
    inputs.voltage = voltageSignal.getValueAsDouble();
    inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
    inputs.velocity = velocitySignal.getValueAsDouble();
  }
}
