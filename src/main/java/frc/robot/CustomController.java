package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CustomController {
  Joystick controller;

  public CustomController(int port) {
    this.controller = new Joystick(port);
  }

  public Trigger button(int button) {
    return new Trigger(() -> controller.getRawButton(button));
  }
}
