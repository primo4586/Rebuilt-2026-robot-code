// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // Pathfinding Constraints
  public static final double PATHFINDER_MAX_VELOCITY = 2.0; // TODO: tune
  public static final double PATHFINDER_MAX_ACCELRATION = 6.0;
  public static final double PATHFINDER_MAX_ANGULAR_VELOCITY = Units.degreesToRadians(540);
  public static final double PATHFINDER_MAX_ANGULAR_ACCELARATION = Units.degreesToRadians(720);

  // positions
  public static final Pose2d side1 = new Pose2d(1.9, 3.9, Rotation2d.fromDegrees(0));
  public static final Pose2d side2 = new Pose2d(3.0, 6.5, Rotation2d.fromDegrees(300));
  public static final Pose2d side3 = new Pose2d(5.6, 6.2, Rotation2d.fromDegrees(240));
  public static final Pose2d side4 = new Pose2d(7.3, 4.0, Rotation2d.fromDegrees(180));
  public static final Pose2d side5 = new Pose2d(5.8, 1.8, Rotation2d.fromDegrees(120));
  public static final Pose2d side6 = new Pose2d(3.2, 1.8, Rotation2d.fromDegrees(60));
  public static final Pose2d[] sides = {side1, side2, side3, side4, side5, side6};

  public static final String CAN_BUS_NAME = "canBus";

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
