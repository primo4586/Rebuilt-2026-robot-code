package frc.robot.subsystems.shooter;

public interface ShooterIO {
  //   @AutoLog
  //   public static class CannonIOInputs {
  //     public double voltage = 0; // in volts
  //     public double current = 0; // in amps
  //   }

  //   default void updateInputs(CannonIOInputs inputs) {
  //     ifOk(motor, voltageSupplier, (value) -> inputs.voltage = value);
  //     ifOk(motor, currentSupplier, (value) -> inputs.current = value);
  //     inputs.opticalSensor = opticalSensorSupplier.getAsBoolean();
  //   }

  //   /**
  //    * Sets the voltage of the motor. Note that we don't use the `motor.setVoltage` method
  // because
  // it
  //    * doesn't seem to work properly with the sim. Instead, we convert the desired voltage to a
  // motor
  //    * output by dividing by the bus voltage. This is done to ensure that the sim works properly.
  //    *
  //    * @param voltage the desired voltage in volts
  //    */
  //   default void setVoltage(double voltage) {
  //     // There is the motor.setVoltage method, but it seems like the sim wont work with it.
  //     motor.set(voltage / motor.getBusVoltage());
  //   }

  //   default void shoot() {
  //     setVoltage(SHOOT_VOLTAGE);
  //   }

  //   /** Disables the motor and stops the cannon from spinning. */
  //   default void stop() {
  //     motor.set(0);
  //   }
}
