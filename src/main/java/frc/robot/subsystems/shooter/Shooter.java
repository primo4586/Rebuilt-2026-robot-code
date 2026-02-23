// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final SysIdRoutine sysId;

  // singleton
  private static Shooter instance;

  public static Shooter getInstance(ShooterIO io) {
    if (instance == null) {
      instance = new Shooter(io);
    }
    return instance;
  }

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;

    // Create the SysId routine
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.setVoltageNoStop(voltage.in(Volts)),
                null, // No log consumer, since data is recorded by AdvantageKit
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  // commands

  /**
   * Set the shooter motors to the given voltage.
   *
   * @param voltage the voltage to set the motor to (in volts)
   * @return a command which sets the voltage and then stops the motor when finished
   */
  public Command setVoltageCommand(double voltage) {
    return startEnd(() -> io.setVoltage(voltage), io::stopMotor)
        .withName(getName() + " Set Voltage");
  }

  public Command setVoltageNoStop(double voltage) {
    return startEnd(() -> io.setVoltage(voltage), io::stopMotor).withName(getName() + "set voltage");
  }

  /**
   * Set the shooter motors to the given current.
   *
   * @param current the current to set the motor to (in amps)
   * @return a command which sets the current and then stops the motor when finished
   */
  public Command setCurrentCommand(double current) {
    return startEnd(() -> io.setCurrent(current), io::stopMotor)
        .withName(getName() + " Set Current");
  }

  /**
   * Set the shooter motors to the given velocity.
   *
   * @param velocity the velocity to set the motor to (in radians per second)
   * @return a command which sets the velocity and then stops the motor when finished
   */
  public Command setVelocityCommand(double velocity) {
    return startEnd(() -> io.setVelocity(velocity), io::stopMotor)
        .withName(getName() + " Set Velocity");
  }

  /**
   * A command which sets the shooter to pass mode and then stops the motor when finished.
   *
   * @return a command which sets the shooter to pass mode and then stops the motor when finished
   */
  public Command passCommand() {
    return startEnd(io::pass, io::stopMotor).withName(getName() + " Pass");
  }

  public double getVoltage() {
    return inputs.voltage;
  }

  public double getStatorCurrent() {
    return inputs.statorCurrent;
  }

  public double getSupplyCurrent() {
    return inputs.supplyCurrent;
  }

  public double getVelocity() {
    return inputs.velocity;
  }

  public double getWantedVelocity() {
    return inputs.wantedVelocity;
  }

  public double getAcceleration() {
    return inputs.acceleration;
  }

  public boolean readyToShoot() {
    return Math.abs(getVelocity() - getWantedVelocity()) < TOLERANCE;
  }

  // sysId Commands
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
