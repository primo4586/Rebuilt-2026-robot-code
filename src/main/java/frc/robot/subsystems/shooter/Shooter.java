// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.primoLib.PrimoCalc;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
                Volts.of(4),
                null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
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
    return startEnd(() -> io.setVoltage(voltage), io::stopMotor)
        .withName(getName() + "set voltage");
  }

  public Command shootWithInterpolation() {
    return setVelocityCommand(PrimoCalc.hubShooterInterpolate());
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
   * @return a command which sets the velocity and then dont stops the motor when finished
   */
  public Command setVelocityCommand(double velocity) {
    return runOnce(() -> io.setVelocity(velocity)).withName(getName() + " Set Velocity");
  }

    /**
   * Set the shooter motors to the given velocity repeatedly.
   *
   * @param velocity the velocity to set the motor to (in radians per second)
   * @return a command which sets the velocity and then dont stops the motor when finished
   */
  public Command setVelocityCommand(DoubleSupplier velocity) {
    return run(() -> io.setVelocity(velocity)).withName(getName() + " Set Velocity");
  }

  /**
   * @param velocity the velocity to set the motor to (in RPS)
   * @return a command that sets voltage to 11.5 until it is within TOLERANCE of the given velocity, then uses velocity control to maintain that velocity. 
   */
  public Command setVoltageWithVelocityCorrection(DoubleSupplier velocity) {
    return run(() -> {if (readyToShoot().getAsBoolean() || getVelocity() > getWantedVelocity()) {
      io.setVelocity(velocity);
    } else {
      io.setVoltage(11.5);
    }}).finallyDo(() -> io.stopMotor()).withName(getName() + " Set Voltage with Velocity Correction");
  }

  /**
   * @return a command that sets velocity with SHOOT_RPS constant
   */
  public Command shoot() {
    return runOnce(() -> io.setVelocity(SHOOT_RPS)).withName(getName() + " Set Velocity");
  }
  /** sets velocity with REST_VELOCITY constant */
  public void rest() {
    io.setVelocity(REST_VELOCITY);
    System.out.println("resting");
  }

  public Command restCommand(){
    return runOnce(this::rest).withName(getName() + " Rest");
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

  public BooleanSupplier readyToShoot() {
    return () -> Math.abs(getVelocity() - getWantedVelocity()) < TOLERANCE;
  }

  // sysId Commands
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
