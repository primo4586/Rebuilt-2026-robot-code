// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private static Feeder instance;

  public static Feeder getInstance(FeederIO io) {
    if (instance == null) {
      instance = new Feeder(io);
    }
    return instance;
  }

  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("feeder", inputs);
  }

  // Commands
  public Command setVoltage(double voltage) {
    return startEnd(() -> io.setVoltage(voltage), () -> io.stopMotor())
        .withName("feeder set voltage");
  }

  public Command setCurrent(double current) {
    return startEnd(() -> io.setCurrent(current), () -> io.stopMotor())
        .withName("feeder set current");
  }
}
