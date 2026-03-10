// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.primoLib.PrimoCalc;

public class Climb extends SubsystemBase {

  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  //singelton
  private static Climb instance;
  public static Climb getInstance(ClimbIO io){
    if(instance != null){
      return instance;
    }
    return new Climb(io);
  }
  /** Creates a new Climb. */
  private Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }

  public Command setVoltageCommand(double voltage) {
    return startEnd(() -> io.setVoltage(voltage), () -> io.setVoltage(0))
        .withName(getName() + " Set Voltage");
  }

  public Command setVoltageNoStop(double voltage) {
    return startEnd(() -> io.setVoltage(voltage), () -> io.setVoltage(0))
        .withName(getName() + "set voltage");
  }

  public Command setPositionCommand(double position) {
    return startEnd(() -> io.relocatePosition(position), () -> io.setVoltage(0))
        .withName(getName() + " Set Position");
  }
  
  public Command resetEncoderPositionCommand(){
    return runOnce(() -> io.setEncoderPosition(0)).withName(getName() + " Reset Encoder Position");
  }

}
