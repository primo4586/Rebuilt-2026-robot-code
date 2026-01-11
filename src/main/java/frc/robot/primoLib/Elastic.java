package frc.robot.primoLib;

import static frc.robot.RobotState.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elastic {
  static Field2d m_field = new Field2d();
  // public static RobotState rState = RobotState.getInstance();

  public static void displayAll() {
    displayRobotPose();
    displayTimer();
    displayField();
  }

  public static void displayTimer() {
    SmartDashboard.putNumber("timer", Timer.getMatchTime());
  }

  public static void displayField() {
    // Do this in either robot or subsystem init
    SmartDashboard.putData("Field", m_field);
  }

  public static void displayRobotPose() {
    // m_field.setRobotPose(Robot POSE); //TODO: where tf do i get the robot pos from help
    // TODO: get good
  }
}
