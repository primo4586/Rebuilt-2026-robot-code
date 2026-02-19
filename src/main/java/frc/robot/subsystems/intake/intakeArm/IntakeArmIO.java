package frc.robot.subsystems.intake.intakeArm;

import static frc.robot.subsystems.intake.intakeArm.IntakeArmConstants.*;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmIO {

  @AutoLog
  public static class IntakeArmIOInputs {
    public double voltage = 0; // in volts
    public double supplyCurrent = 0; // in amps
    public double statorCurrent = 0; // in amps
    public double velocity = 0; // in rps
    public double position = 0; // in rotations
  }

  public default void updateInputs(IntakeArmIOInputs inputs) {
  }

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
