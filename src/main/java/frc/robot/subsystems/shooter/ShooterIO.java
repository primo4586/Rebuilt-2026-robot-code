package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double voltage = 0; // in volts 
    public double supplyCurrent = 0; // in amps
    public double statorCurrent = 0; // in amps
    public double velocity = 0; // in rps
    public double acceleration = 0; // in rps^2
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setCurrent(double current) {}

  public default void setVelocity(double velocity){}

  public default void stopMotor(double velocity){}

}
