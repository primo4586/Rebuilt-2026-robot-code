package frc.robot.subsystems.feeder;

import static frc.robot.subsystems.feeder.FeederConstants.*;
import static frc.robot.subsystems.feeder.FeederConstants.FeederSimConstants.*;

import com.ctre.phoenix6.StatusSignal;

public class FeederSim implements FeederIO {

  public FeederSim() {
    _motor.getConfigurator().apply(simConfigs);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {

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
