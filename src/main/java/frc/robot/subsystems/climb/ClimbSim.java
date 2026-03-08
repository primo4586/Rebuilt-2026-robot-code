package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;

import static frc.robot.subsystems.climb.ClimbConstants.ClimbSimConstants.*;

public class ClimbSim implements ClimbIO {
    public ClimbSim() {
        _motor.getConfigurator().apply(simConfiguration);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        //supply voltage
        simMotor.setSupplyVoltage(12);
        //stator voltage
        elevatorSim.setInputVoltage(simMotor.getMotorVoltage());

        //sim update rate
        elevatorSim.update(0.02);
    }


}
