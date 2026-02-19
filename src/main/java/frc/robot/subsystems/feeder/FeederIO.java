package frc.robot.subsystems.feeder;

import static frc.robot.subsystems.feeder.FeederConstants.*;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

  @AutoLog
  public static class FeederIOInputs {
    public double voltage = 0; // in volts
    public double supplyCurrent = 0; // in amps
    public double statorCurrent = 0; // in amps
    public double velocity = 0; // in rps
  }

  public default void updateInputs(FeederIOInputs inputs) {
  }

  public default void setVoltage(double voltage) {
    _motor.set(voltage / _motor.getSupplyVoltage().getValueAsDouble());
  }

  public default void setCurrent(double current) {
    _motor.setControl(currentRequest.withOutput(current));
  }

  public default void stopMotor() {
    _motor.stopMotor();
  }
}
