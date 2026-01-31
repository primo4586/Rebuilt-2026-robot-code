// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeArm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeArm extends SubsystemBase {

  private final IntakeArmIO io;
  private final IntakeArmIOInputsAutoLogged inputs = new IntakeArmIOInputsAutoLogged();

  private static IntakeArm instance;

  public static IntakeArm getInstance(IntakeArmIO io) {
    if (instance == null) {
      instance = new IntakeArm(io);
    }
    return instance;
  }

  public IntakeArm(IntakeArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeArm", inputs);
  }

  // Commands
  public Command setVoltage(double voltage) {
    return runOnce(() -> io.setVoltage(voltage)).withName("IntakeArmSetVoltage");
  }

  public Command setCurrent(double current) {
    return runOnce(() -> io.setCurrent(current)).withName("IntakeArmSetCurrent");
  }

  public Command setPosition(double position) {
    return runOnce(() -> io.setPosition(position)).withName("IntakeArmSetPosition");
  }
}
