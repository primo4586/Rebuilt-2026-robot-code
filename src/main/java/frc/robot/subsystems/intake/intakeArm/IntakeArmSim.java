package frc.robot.subsystems.intake.intakeArm;

import static frc.robot.subsystems.intake.intakeArm.IntakeArmConstants.*;
import static frc.robot.subsystems.intake.intakeArm.IntakeArmConstants.IntakeArmSimConstants.*;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeArmSim implements IntakeArmIO {

  public IntakeArmSim() {
    _motor.getConfigurator().apply(simConfiguration);
  }

  @Override
  public void updateInputs(IntakeArmIOInputs inputs) {

    // supply voltage
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());

    // stator voltage
    armSim.setInputVoltage(simMotor.getMotorVoltage());

    // sim update rate
    armSim.update(0.02);

    // Calculate Mechanism Position (The physical arm)
    double mechanismPosition = armSim.getAngleRads() / (2 * Math.PI);
    double mechanismVelocity = armSim.getVelocityRadPerSec() / (2 * Math.PI);

    // Calculate Rotor Position (The motor shaft) -> MULTIPLY by Gear Ratio
    double rotorPosition = mechanismPosition * GEAR_RATIO;
    double rotorVelocity = mechanismVelocity * GEAR_RATIO;

    // Set Sim State (SimState always expects Rotor Rotations)
    simMotor.setRawRotorPosition(rotorPosition);
    simMotor.setRotorVelocity(rotorVelocity);

    // set sim roborio voltage
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    inputs.position = mechanismPosition;
    inputs.velocity = mechanismVelocity;

    inputs.voltage = simMotor.getMotorVoltage();
    inputs.statorCurrent = simMotor.getTorqueCurrent();
    inputs.supplyCurrent = simMotor.getSupplyCurrent();

    // Mechanism 2d (Visualizer expects degrees of the arm)
    arm.setAngle(mechanismPosition * 360);
    SmartDashboard.putData("Mech2d", mech2d);
  }
}
