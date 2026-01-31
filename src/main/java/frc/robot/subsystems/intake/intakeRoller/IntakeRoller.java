// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase {

  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  private static IntakeRoller instance;

  public static IntakeRoller getInstance(IntakeRollerIO io) {
    if (instance == null) {
      instance = new IntakeRoller(io);
    }
    return instance;
  }

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  // Commands
  public Command setVoltage(double voltage) {
    return runOnce(() -> io.setVoltage(voltage)).withName("IntakeRollerSetVoltage");
  }

  public Command setCurrent(double current) {
    return runOnce(() -> io.setCurrent(current)).withName("IntakeRollerSetCurrent");
  }
}
