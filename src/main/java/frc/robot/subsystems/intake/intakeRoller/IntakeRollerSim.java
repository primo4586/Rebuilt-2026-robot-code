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
        voltageSignal, supplyCurrentSignal, statorCurrentSignal, velocitySignal);

    // updates inputs
    inputs.voltage = voltageSignal.getValueAsDouble();
    inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
    inputs.velocity = velocitySignal.getValueAsDouble();
  }
}
