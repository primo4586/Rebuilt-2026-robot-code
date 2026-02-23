// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.targetPosition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final SysIdRoutine sysId;

  private static Hood instance;

  public static Hood getInstance(HoodIO io) {
    if (instance == null) {
      instance = new Hood(io);
    }
    return instance;
  }

  public Hood(HoodIO io) {
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
    Logger.processInputs("Hood", inputs);
  }

  // Commands
  public Command setVoltage(double voltage) {
    return startEnd(() -> io.setVoltage(voltage), io::stopMotor).withName(getName() + "Set voltage");
  }

  public Command setVoltageNoStop(double voltage) {
    return startEnd(() -> io.setVoltage(voltage), io::stopMotor).withName(getName() + "Set voltage-noStop");
  }

  public Command setCurrent(double current) {
    return startEnd(() -> io.setCurrent(current), io::stopMotor).withName(getName() + "Set current");
  }

  public Command setPosition(double position) {
    return runOnce(() -> io.setPosition(position)).withName(getName() + "Set position");
  }

  public Command followTargetPosition() { // todo: check if position is updated
    return run(() -> io.setPosition(targetPosition.getAsDouble()))
        .withName(getName() + "Follow target position");
  }

  // sysid commands
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  // sysid commands
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
