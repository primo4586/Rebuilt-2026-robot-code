package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

      @AutoLog
  public static class ClimbIOInputs {
    public double voltage = 0; // in volts
    public double supplyCurrent = 0; // in amps
    public double statorCurrent = 0; // in amps
    public double postison = 0; // in rotations
    public double velocity = 0; // in rps
    public double accelertion = 0; // in rps^2
  }
    default void updateInputs(ClimbIOInputs inputs){};

    default void setVoltage(double voltage){
        _motor.set(voltage / _motor.getSupplyVoltage().getValueAsDouble());
    };
    default void setCurrent(double current){
        _motor.setControl(currentRequest.withOutput(current));
    }
        default void relocatePostiton(double postiton){
        _motor.setControl(positionRequest.withPosition(postiton));
    }
        default void setVelocity(double velocity){
        _motor.setControl(velocityRequest.withVelocity(velocity));
    }
    
} 
