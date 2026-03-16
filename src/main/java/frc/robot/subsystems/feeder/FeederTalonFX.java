package frc.robot.subsystems.feeder;

import static frc.robot.subsystems.feeder.FeederConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

public class FeederTalonFX implements FeederIO {

  public FeederTalonFX() {
    
    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      _motor.getConfigurator().apply(realConfigs);
      if (statusCode.isOK()) {
        break;
      }
    }
    if (!statusCode.isOK()) {
      System.out.println("feeder configs failed" + statusCode.toString());
    }
      SPARKMAX_configs.smartCurrentLimit(SPARKMAX_MAX_CURRENT).idleMode(SPARKMAX_IDLE_MODE).inverted(SPARKMAX_INVERTED);

    tryUntilOk(
        _SparkMax,
        5,
        () ->
          _SparkMax.configure(
                SPARKMAX_configs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    StatusSignal.refreshAll(
        voltageSignal, supplyCurrentSignal, statorCurrentSignal, velocitySignal);
    inputs.voltage = voltageSignal.getValueAsDouble();
    inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
    inputs.velocity = velocitySignal.getValueAsDouble();
  }
}
