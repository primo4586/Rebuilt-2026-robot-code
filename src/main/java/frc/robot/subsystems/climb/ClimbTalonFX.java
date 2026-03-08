package frc.robot.subsystems.climb;

import com.ctre.phoenix6.StatusCode;
import static frc.robot.subsystems.climb.ClimbConstants.*;
public class ClimbTalonFX implements ClimbIO{
    
    public ClimbTalonFX() {

    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      _motor.getConfigurator().apply(realConfiguration);
      if (statusCode.isOK()) {
        break;
      }
    }
    if (statusCode.isError()) {
      System.out.println("Climb configs failed" + statusCode.toString());
    }
  }
}
