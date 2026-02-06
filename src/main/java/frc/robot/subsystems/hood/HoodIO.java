package frc.robot.subsystems.hood;

import static frc.robot.subsystems.hood.HoodConstants.*;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public double voltage = 0; // in volts
    public double supplyCurrent = 0; // in amps
    public double statorCurrent = 0; // in amps
    public double velocity = 0; // in rps
    public double position = 0; // in rotations
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setVoltage(double voltage) {
    _motor.set(voltage / _motor.getSupplyVoltage().getValueAsDouble());
  }

  public default void setCurrent(double current) {
    _motor.setControl(currentRequest.withOutput(current));
  }

  public default void setPosition(double position) {
    _motor.setControl(positionRequest.withPosition(position));
  }

  public default void stopMotor() {
    _motor.stopMotor();
  }
}
