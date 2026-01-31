package frc.robot.subsystems.intake.intakeRoller;

import static frc.robot.subsystems.intake.intakeRoller.IntakeRollerConstants.*;
import static frc.robot.subsystems.intake.intakeRoller.IntakeRollerConstants.intakeRollerSimConstants.*;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class IntakeRollerSim implements IntakeRollerIO {

  public IntakeRollerSim() {
    _motor.getConfigurator().apply(configuration);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {

    // supply voltage
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());

    // stator voltage
    sim.setInputVoltage(simMotor.getMotorVoltage());

    // sim update rate
    sim.update(0.02);

    // calculate the motor's velocity
    double velocity = sim.getAngularVelocityRadPerSec() / (2 * Math.PI) * GEAR_RATIO;

    // apply the values
    simMotor.setRotorVelocity(velocity);

    // set sim roborio voltage
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

    // update inputs
    inputs.voltage = simMotor.getMotorVoltage();
    inputs.statorCurrent = simMotor.getTorqueCurrent();
    inputs.supplyCurrent = simMotor.getSupplyCurrent();
    inputs.velocity = velocity;
  }
}
