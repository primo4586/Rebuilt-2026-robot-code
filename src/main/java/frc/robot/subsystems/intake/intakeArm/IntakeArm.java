// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeArm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.littletonrobotics.junction.Logger;

public class IntakeArm extends SubsystemBase {

  private final IntakeArmIO io;
  private final IntakeArmIOInputsAutoLogged inputs = new IntakeArmIOInputsAutoLogged();

  private final SysIdRoutine sysId;

  private static IntakeArm instance;

  public static IntakeArm getInstance(IntakeArmIO io) {
    if (instance == null) {
      instance = new IntakeArm(io);
    }
    return instance;
  }

  public IntakeArm(IntakeArmIO io) {
    this.io = io;

    // Create the SysId routine
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, null, null, // Use default config
            (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> this.setVoltageNoStop(voltage.in(Volts)),
            null, // No log consumer, since data is recorded by AdvantageKit
            this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeArm", inputs);
  }

  // Commands
  public Command setVoltage(double voltage) {
    return startEnd(() -> io.setVoltage(voltage), io::stopMotor).withName("IntakeArmSetVoltage");
  }

  public Command setVoltageNoStop(double voltage) {
    return startEnd(() -> io.setVoltage(voltage), io::stopMotor).withName("hood set voltage");
  }

  public Command setCurrent(double current) {
    return startEnd(() -> io.setCurrent(current), io::stopMotor).withName("IntakeArmSetCurrent");
  }

  public Command setPosition(double position) {
    return runOnce(() -> io.setPosition(position)).withName("IntakeArmSetPosition");
  }

  // sysId Commands

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  // sysid commands
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

}
