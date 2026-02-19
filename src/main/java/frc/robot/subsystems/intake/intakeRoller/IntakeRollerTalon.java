package frc.robot.subsystems.intake.intakeRoller;

import static frc.robot.subsystems.intake.intakeRoller.IntakeRollerConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

public class IntakeRollerTalon implements IntakeRollerIO {

  public IntakeRollerTalon() {

    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      _motor.getConfigurator().apply(realConfigs);
      if (statusCode.isOK()) {
        break;
      }
    }
    if (statusCode.isError()) {
      System.out.println("intake roller configs failed" + statusCode.toString());
    }
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    StatusSignal.refreshAll(
        voltageSignal, supplyCurrentSignal, statorCurrentSignal, velocitySignal);
    inputs.voltage = voltageSignal.getValueAsDouble();
    inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
    inputs.velocity = velocitySignal.getValueAsDouble();
  }
}
