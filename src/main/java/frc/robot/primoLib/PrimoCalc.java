package frc.robot.primoLib;

public class PrimoCalc {

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
}
