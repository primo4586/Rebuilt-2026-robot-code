package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.primoLib.PrimoCalc;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.hood.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterRealIO;
import frc.robot.subsystems.shooter.ShooterSimIO;

public class CommandGroupFactory {
  private static final Drive drive = Drive.getInstance(RobotBase.isReal());
  private static final Shooter shooter =
      Shooter.getInstance(RobotBase.isReal() ? new ShooterRealIO() : new ShooterSimIO());
  private static final Hood hood =
      Hood.getInstance(RobotBase.isReal() ? new HoodTalon() : new HoodSim());
  public static final Feeder feeder =
      Feeder.getInstance(RobotBase.isReal() ? new FeederTalonFX() : new FeederSim());

  /**
   * turn to hub, stop with x, and shoot with interpolation
   */
  public static Command shootCommand() {
    return Commands.sequence(
        Commands.parallel(
            hood.setPositionWithInterpolation(),
            shooter.shoot(),
            Commands.deadline(
                Commands.waitUntil(PrimoCalc.isFacingHub()),
                printCommand(PrimoCalc.getRadsToHub()).repeatedly(),
                DriveCommands.joystickDriveAtAngle(
                        drive, () -> 0, () -> 0, () -> (new Rotation2d(PrimoCalc.getRadsToHub())))
                    .repeatedly())),
        Commands.parallel(
            Commands.runOnce(() -> drive.stopWithX()), feeder.feed()));
  }

  /**
   * returns System.out.println as a command
   *
   * @return System.out.println as a command
   * @param s string to print
   */
  public static Command printCommand(String output) {
    return Commands.runOnce(() -> System.out.println(output));
  }

  /**
   * returns System.out.println as a command
   *
   * @return System.out.println as a command
   * @param d double to print
   */
  public static Command printCommand(double output) {
    return Commands.runOnce(() -> System.out.println(output));
  }
}
