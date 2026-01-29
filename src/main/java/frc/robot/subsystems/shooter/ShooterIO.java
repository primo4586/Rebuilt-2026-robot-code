package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double voltage = 0; // in volts
    public double supplyCurrent = 0; // in amps
    public double statorCurrent = 0; // in amps
    public double velocity = 0; // in rps
    public double acceleration = 0; // in rps^2
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltage(double voltage) {
    _masterMotor.set(voltage/_masterMotor.getSupplyVoltage().getValueAsDouble());
  }

  public default void setCurrent(double current) {
    _masterMotor.setControl(currentRequest.withOutput(current));
  }

  public default void setVelocity(double velocity) {
    _masterMotor.setControl(velocityRequest.withVelocity(velocity));
  }

  public default void stopMotor() {
    _masterMotor.stopMotor();
  }

  public default double caculateWantedSpeed(){
    /*⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡤⠖⠉⣉⣉⣉⡉⠉⠉⠒⠢⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡔⢋⣠⣶⠿⣛⡯⠭⠿⢟⣳⠶⣄⠠⠝⢆⠀⠀⠀⠀⠀⠀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡴⠉⣰⣾⢏⠵⠋⠁⠀⠀⠀⠀⠈⠙⢮⡢⢸⣮⢣⣀⠴⢒⣯⣍⡉⠉⠲⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⡞⠁⣼⡿⣱⠋⠀⠀⠀⠀⠀⠀⣀⡤⠤⠖⣻⣿⣿⣿⣿⣾⣿⣿⣏⣛⡛⠦⣜⡂⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢀⡖⣫⣿⠁⣼⣿⢱⠋⠀⠀⠀⠀⠀⡴⠋⢰⣶⡿⣿⣿⣿⣿⣿⢿⡿⣿⣿⣿⣿⣿⣿⣶⣝⠢⡀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠈⠉⢻⣿⡆⣿⣏⡏⠀⠀⠀⠀⠀⣸⠃⣰⢏⣵⣿⣿⣻⢯⡷⣯⣟⣿⣷⡿⣿⣿⣿⣿⣿⣿⣷⡱⡄⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡏⠅⢺⣷⡇⠀⠀⠀⠀⠀⣿⣠⢯⢿⣿⣟⡾⣽⢯⣟⣷⣻⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢳⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠁⢺⣿⢳⠀⠀⠀⠀⠀⠉⠁⢸⢺⣿⢾⣽⢯⣿⡽⣾⢷⣿⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣏⢇⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠀⢹⣿⡼⢗⣛⣏⣷⡄⠀⠀⢸⣹⣿⣯⣿⣿⢾⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣎⢦⡀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡷⡇⠀⢡⣰⡾⣫⠞⠋⠁⠀⠀⠀⠙⣿⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣧⢇⠀⣤⣿⢿⠁⠀⠀⠀⠀⠀⠀⣰⣻⣿⣳⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢿⣹⠀⠸⣿⡞⡇⠀⠀⠀⠀⠀⠀⣿⣿⣳⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⣟⡇⠀⢿⣷⣸⡀⠀⠀⠀⠀⣰⢟⣯⢿⡾⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⡇⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣷⢻⠀⡘⣿⣇⢧⠀⠀⠀⠀⢷⡯⣟⣯⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⡿⠿⠛⠉⠀⠀⠀
⠀⠀⠀⠀⠀⢀⣀⣀⣀⠀⠀⠸⡽⡇⠠⢹⣿⡌⣇⠀⠀⠀⠀⠈⠉⠉⠉⠙⠿⣻⣿⠟⢯⣿⣽⠿⢿⣿⡿⠿⠚⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⣠⠴⠊⠙⢿⢳⣫⢍⡲⢴⣷⢿⡄⠡⢿⣿⡘⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠉⠉⠉⠉⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⢀⡼⢁⠤⡠⠀⠀⠓⣌⠓⢝⠶⣽⣛⠎⡀⢭⣿⣷⡹⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⢸⠰⣏⢲⣅⢃⠀⣀⠈⢧⡀⠱⣝⢿⣿⣶⣹⢾⣿⣧⢹⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡰⣲⡲⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⣸⢸⣯⢖⣯⢾⡽⣶⣻⡎⡇⠀⠈⢳⣻⣿⣳⢿⣿⣿⣇⢳⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡸⣿⣿⣗⣇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⢾⣨⣿⡜⣯⣿⢿⣿⣿⡇⣷⠀⠀⠈⢧⢻⣯⣿⣿⣿⣿⡎⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⡜⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠈⢳⡹⣷⡹⣾⣿⣿⣿⣧⣿⠀⠀⠀⠘⡟⣿⣿⣯⣿⣿⣷⢻⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠐⣿⣿⣿⣿⣿⣞⢆⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠱⡝⣿⡽⣿⣿⢏⡜⠁⠀⠀⠀⠀⢽⢹⣿⣿⣿⣿⣿⠼⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢟⣿⣿⣿⣿⣿⣧⣕⠦⣀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⢸⢽⡿⢣⠎⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⡃⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⢿⢿⣿⣻⡿⣿⡿⣿⣾⣿⣯⣖⣒⡀
⠀⠀⠀⠀⠘⣪⠖⠁⠀⠀⠀⠀⠀⠀⠀⢸⢷⣿⣿⣿⣿⣿⡁⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢿⣻⣽⣷⣿⣳⣯⣟⣷⣟⡿⠛
⠀⠀⠀⠈⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⢸⡿⣽⣾⢿⣿⢀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠒⠒⠒⠋⠉⠀⠀⠀ */
    return 0; 
  }
}
