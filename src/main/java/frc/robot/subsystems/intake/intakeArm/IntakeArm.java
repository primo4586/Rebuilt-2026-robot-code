// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeArm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import static frc.robot.subsystems.intake.intakeArm.IntakeArmConstants.*;

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
    resetPosition();

    // Create the SysId routine
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null, // Use default config
            (state) -> Logger.recordOutput("ShooterSysIdTestState", state.toString())),
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

  public void resetPosition() {
    io.resetPosition();
  }

  // Commands
  /**
   * @return a command that sets the voltage of the intake arm motor. The motor will be stopped when the command ends. 
   */
  public Command setVoltage(double voltage) {
    return startEnd(() -> io.setVoltage(voltage), io::stopMotor)
        .withName(getName() + "Set voltage");
  }

  /**
   * @return Set the voltage of the intake arm motor without stopping it when the command ends. 
   */
  public Command setVoltageNoStop(double voltage) {
    return startEnd(() -> io.setVoltage(voltage), io::stopMotor)
        .withName(getName() + "Set voltage");
  }

  /**
   * @return a command that sets the current of the intake arm motor. The motor will be stopped when the command ends.
   */
  public Command setCurrent(double current) {
    return startEnd(() -> io.setCurrent(current), io::stopMotor)
        .withName(getName() + "Set current");
  }

  /**
   * @return a command that sets the position of the intake arm.
   */
  public Command setPositionCommand(double position) {
    return runOnce(() -> io.setPosition(position)).withName(getName() + "Set position");
  }

  /**
   * @return a command that opens the intake arm. 
   */
  public Command openCommand(){
    return setPositionCommand(OPEN_POSITION);
  }

  /**
   * @return a command that closes the intake arm.
   */
  public Command closeCommand(){
    return setPositionCommand(CLOSE_POSITION);
  }

  /**
   * @return a command that repeatedly opens and closes the intake arm with a 1 second delay in between. The intake arm will be left open when the command ends. for shoot commands.
   */
  public Command openAndCloseCommand(){
    return Commands.repeatingSequence(
      openCommand(),
      Commands.waitSeconds(1),
      closeCommand(),
      Commands.waitSeconds(1)).finallyDo(() -> io.setPosition(OPEN_POSITION)).withName(getName() + "Open and Close");
  }

  /**
   * @return a command that resets the position of the intake arm. for manual resetting. 
   */
  public Command resetPositionCommand(){
    return setVoltage(RESET_VOLTAGE).finallyDo(() -> io.resetPosition());
  }

  // sysId Commands

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
