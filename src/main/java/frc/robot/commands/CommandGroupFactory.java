package frc.robot.commands;

import static frc.robot.Constants.*;
import static frc.robot.RobotState.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.util.interpolation.InterpolateUtil;

public class CommandGroupFactory {
  private final Drive drive = Drive.getInstance(RobotBase.isReal());

  public static Pose2d getHubPos(){
  return DriverStation.getAlliance().isPresent()
  && DriverStation.getAlliance().get() == Alliance.Red
      ? new Pose2d(4.5, 4, new Rotation2d())
      : new Pose2d(12, 4, new Rotation2d());
  }

  public static double getDistance(Pose2d a, Pose2d b) {
    return a.getTranslation().getDistance(b.getTranslation());
  }

  // interpolation
  public double hubHoodInterpolate() {
    return InterpolateUtil.interpolate(
        HoodConstants.HOOD_ANGLE_INTERPOLATION_MAP,
        getDistance(getHubPos(), drive.getPose()));
  }

  public double hubShooterInterpolate() {
    return InterpolateUtil.interpolate(
        ShooterConstants.SHOOTER_INTERPOLATION_MAP,
        getDistance(getHubPos(), drive.getPose()));
  }

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
