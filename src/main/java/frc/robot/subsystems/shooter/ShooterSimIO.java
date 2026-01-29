package frc.robot.subsystems.shooter;
import static frc.robot.subsystems.shooter.ShooterConstants.ShooterSimConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
public class ShooterSimIO implements ShooterIO {
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // gets deta
        simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
        sim.setInputVoltage(MathUtil.clamp(simMotor.getMotorVoltage(), -12.0, 12.0));
        sim.update(0.02); 

        // updates inputs 
        inputs.voltage = sim.getInputVoltage();
        inputs.statorCurrent = simMotor.getTorqueCurrent();
        inputs.supplyCurrent = sim.getCurrentDrawAmps();
        inputs.velocity = sim.getAngularVelocityRadPerSec();
        //^ there is no acceleration in the sim
    }
    }
