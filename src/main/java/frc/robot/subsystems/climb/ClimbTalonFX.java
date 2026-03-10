package frc.robot.subsystems.climb;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;

import static frc.robot.subsystems.climb.ClimbConstants.*;
public class ClimbTalonFX implements ClimbIO{
    
    public ClimbTalonFX() {

    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      _motor.getConfigurator().apply(realConfiguration);
      if (statusCode.isOK()) {
        break;
      }
    }
    if (statusCode.isError()) {
      System.out.println("Climb configs failed" + statusCode.toString());
    }
  }

    @Override
  public void updateInputs(ClimbIOInputs inputs) {
    StatusSignal.refreshAll(
        voltageSignal, supplyCurrentSignal, statorCurrentSignal, velocitySignal, positionSignal, AccelerationSignal);
    inputs.voltage = voltageSignal.getValueAsDouble();
    inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
    inputs.velocity = velocitySignal.getValueAsDouble();
    inputs.postison = positionSignal.getValueAsDouble();
    inputs.accelertion = AccelerationSignal.getValueAsDouble();
  }
}
