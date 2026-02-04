package frc.robot.subsystems.intake.intakeRoller;

import static frc.robot.subsystems.intake.intakeRoller.IntakeRollerConstants.*;
import static frc.robot.subsystems.intake.intakeRoller.IntakeRollerConstants.intakeRollerSimConstants.*;

import com.ctre.phoenix6.StatusSignal;

public class IntakeRollerSim implements IntakeRollerIO {

  public IntakeRollerSim() {
    _motor.getConfigurator().apply(simConfigs);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {

    // supply voltage
    simMotor.setSupplyVoltage(12);

    // update sim mechanism
    sim.setInputVoltage(_motor.getMotorVoltage().getValueAsDouble());
    sim.update(0.02);

    // update motor
    simMotor.setRotorVelocity(sim.getAngularVelocity().times(GEAR_RATIO));
    simMotor.setRotorAcceleration(sim.getAngularAcceleration().times(GEAR_RATIO));

    // update status ctre
    StatusSignal.refreshAll(
        currentVoltage, currentSupplyCurrent, currentStatorCurrent, currentVelocity);

    // updates inputs
    inputs.voltage = currentVoltage.getValueAsDouble();
    inputs.statorCurrent = currentStatorCurrent.getValueAsDouble();
    inputs.supplyCurrent = currentSupplyCurrent.getValueAsDouble();
    inputs.velocity = currentVelocity.getValueAsDouble();
  }
}
