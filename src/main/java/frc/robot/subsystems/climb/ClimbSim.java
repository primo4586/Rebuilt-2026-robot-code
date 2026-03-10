package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;

import static frc.robot.subsystems.climb.ClimbConstants.ClimbSimConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbSim implements ClimbIO {
    public ClimbSim() {
        _motor.getConfigurator().apply(simConfiguration);
    }

 @Override
  public void updateInputs(ClimbIOInputs inputs) {
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());

    elevatorSim.setInputVoltage(simMotor.getMotorVoltage());
    elevatorSim.update(0.02);

    double position = elevatorSim.getPositionMeters();
    double velocity = elevatorSim.getVelocityMetersPerSecond();

    simMotor.setRawRotorPosition(position / (2 * Math.PI ) * GEAR_RATIO);
    simMotor.setRotorVelocity(velocity / (2 * Math.PI ) * GEAR_RATIO);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    BaseStatusSignal.refreshAll(
        voltageSignal,
        statorCurrentSignal,
        supplyCurrentSignal,
        positionSignal,
        velocitySignal);

    inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
    inputs.voltage = voltageSignal.getValueAsDouble();
    inputs.postison = positionSignal.getValueAsDouble();
    inputs.velocity = velocitySignal.getValueAsDouble();

    // Mechanism 2d
    climb2d.setLength(position);
    SmartDashboard.putData("Mech2d", mech2d);
  }


}
