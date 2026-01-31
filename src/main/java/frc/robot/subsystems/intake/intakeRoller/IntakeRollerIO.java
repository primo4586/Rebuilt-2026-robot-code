package frc.robot.subsystems.intake.intakeRoller;

import static frc.robot.subsystems.intake.intakeRoller.IntakeRollerConstants.*;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusSignal;

public interface IntakeRollerIO {

  @AutoLog
  public static class IntakeRollerIOInputs {
    public double voltage = 0; // in volts
    public double supplyCurrent = 0; // in amps
    public double statorCurrent = 0; // in amps
    public double velocity = 0; // in rps
    public double acceleration = 0; // in rps^2
  }

  public default void updateInputs(IntakeRollerIOInputs inputs) {}

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
