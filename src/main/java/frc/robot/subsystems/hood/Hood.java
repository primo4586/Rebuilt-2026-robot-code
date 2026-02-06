// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import static frc.robot.subsystems.hood.HoodConstants.targetPosition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private static Hood instance;

  public static Hood getInstance(HoodIO io) {
    if (instance == null) {
      instance = new Hood(io);
    }
    return instance;
  }

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  // Commands
  public Command setVoltage(double voltage) {
    return startEnd(() -> io.setVoltage(voltage), io::stopMotor).withName("hood set voltage");
  }

  public Command setCurrent(double current) {
    return startEnd(() -> io.setCurrent(current), io::stopMotor).withName("hood set current");
  }

  public Command setPosition(double position) {
    return runOnce(() -> io.setPosition(position)).withName("hood set position");
  }

  public Command followTargetPosition() { // todo: check if position is updated
    return run(() -> io.setPosition(targetPosition.getAsDouble()))
        .withName("hood follow target position");
  }
}
