package frc.robot.subsystems.intake.intakeRoller;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import static frc.robot.subsystems.intake.intakeRoller.IntakeRollerConstants.*;

public class IntakeRollerTalon implements IntakeRollerIO {

  public IntakeRollerTalon() {

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
    public void updateInputs(IntakeRollerIOInputs inputs) {
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
