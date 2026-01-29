package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class ShooterTalonFX implements ShooterIO {
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        StatusSignal.refreshAll(currentVoltage, currentSupplyCurrent,currentStatorCurrent, currentVelocity, currentAcceleration);
        inputs.voltage = currentVoltage.getValueAsDouble();
        inputs.statorCurrent = currentStatorCurrent.getValueAsDouble();
        inputs.supplyCurrent = currentSupplyCurrent.getValueAsDouble();
        inputs.velocity = currentVelocity.getValueAsDouble();
        inputs.acceleration = currentAcceleration.getValueAsDouble();
    }
}
