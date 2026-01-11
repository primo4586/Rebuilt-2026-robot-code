package frc.robot.commands;

import static frc.robot.Constants.*;
import static frc.robot.RobotState.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CommandGroupFactory {

  /**
   * returns System.out.println as a command
   *
   * @return System.out.println as a command
   * @param s string to print
   */
  public static Command printCommand(String s) {
    return Commands.runOnce(() -> System.out.println(s));
  }
}
