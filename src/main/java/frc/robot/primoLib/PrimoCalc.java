package frc.robot.primoLib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.interpolation.InterpolateUtil;
import java.util.function.BooleanSupplier;

public class PrimoCalc {
  private static final Drive drive = Drive.getInstance(RobotBase.isReal());

  /**
   * Snaps an angle to the nearest value of the form 45 + 90k and normalizes the result to the range
   * [0, 360].
   *
   * @param angle input angle in degrees
   * @return nearest snapped angle in [0, 360]
   */
  public static double bumpAngle(double angle) {
    double snapped = 45 + 90 * Math.round((angle - 45) / 90.0);
    return (snapped % 360 + 360) % 360;
  }

  /**
   * @return hub coordinates as a pose3d
   */
  public static Pose3d getHubPos() {
    return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Blue
        ? new Pose3d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84),
                Units.inchesToMeters(72), Rotation3d.kZero)
        : new Pose3d(11.912, Units.inchesToMeters(158.84),
        Units.inchesToMeters(72), Rotation3d.kZero);
  }

  /**
   * @return the angle the robot needs to be to point at the hub, on radians.
   */
  public static double getRadsToHub() {
    return drive
            .getPose()
            .getTranslation()
            .minus(getHubPos().toPose2d().getTranslation())
            .getAngle()
            .getRadians()
        + Math.PI;
  }

  /**
   * @param targetPose the pose to point at
   * @return returns the angle the robot needs to be to point at a given pose, in radians.
   */
  public static double getRadsToPose(Pose2d targetPose) {
    return drive
            .getPose()
            .getTranslation()
            .minus(targetPose.getTranslation())
            .getAngle()
            .getRadians()
        + Math.PI;
  }

/**
 * Checks if the robot is facing the hub within a certain threshold.
 */
  public static BooleanSupplier isFacingHub() {
    return () ->
        Math.abs(getRadsToHub() - drive.getRotation().getRadians())
            < Constants.HUB_ANGLE_RADS_THRESHOLD;
  }

  public static double getDistance(Pose2d a, Pose3d b) {
    return a.getTranslation().getDistance(b.toPose2d().getTranslation());
  }

  // interpolation
  /**
   * @return the hood angle in radians to hit the hub based on the robot's current distance from the hub
  */
  public static double hubHoodInterpolate() {
    return InterpolateUtil.interpolate(
        HoodConstants.HOOD_ANGLE_INTERPOLATION_MAP, getDistance(drive.getPose(), getHubPos()));
  }

    /**
    * @return the shooter wheel speed in RPS to hit the hub based on the robot's current distance from the hub
    */
  public static double hubShooterInterpolate() {
    return InterpolateUtil.interpolate(
        ShooterConstants.SHOOTER_INTERPOLATION_MAP, getDistance(drive.getPose(), getHubPos()));
  }
}
