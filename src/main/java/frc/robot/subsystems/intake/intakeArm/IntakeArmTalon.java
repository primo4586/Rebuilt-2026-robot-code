package frc.robot.subsystems.intake.intakeArm;

import static frc.robot.subsystems.intake.intakeArm.IntakeArmConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

public class IntakeArmTalon implements IntakeArmIO {

  public IntakeArmTalon() {

    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      _motor.getConfigurator().apply(realConfiguration);
      if (statusCode.isOK()) {
        break;
      }
    }
    if (statusCode.isError()) {
      System.out.println("Cannon configs failed" + statusCode.toString());
    }
  }

  @Override
  public void updateInputs(IntakeArmIOInputs inputs) {
    StatusSignal.refreshAll(
        currentVoltage, currentSupplyCurrent, currentStatorCurrent, currentVelocity);
    inputs.voltage = currentVoltage.getValueAsDouble();
    inputs.statorCurrent = currentStatorCurrent.getValueAsDouble();
    inputs.supplyCurrent = currentSupplyCurrent.getValueAsDouble();
    inputs.velocity = currentVelocity.getValueAsDouble();
  }
}
