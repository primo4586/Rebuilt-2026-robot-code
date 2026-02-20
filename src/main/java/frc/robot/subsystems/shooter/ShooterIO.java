package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double voltage = 0; // in volts
    public double supplyCurrent = 0; // in amps
    public double statorCurrent = 0; // in amps
    public double velocity = 0; // in rps
    public double wantedVelocity = 0; // in rps
    public double acceleration = 0; // in rps^2
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltage(double voltage) {
    _masterMotor.set(
        voltage
            / _masterMotor
                .getSupplyVoltage()
                .getValueAsDouble()); // set voltage done not work well in simulation from my
    // experience
  }

  public default void setCurrent(double current) {
    _masterMotor.setControl(currentRequest.withOutput(current));
  }

  public default void setVelocity(double velocity) {
    _masterMotor.setControl(velocityRequest.withVelocity(velocity));
    targetVelocity = velocity;
  }

  /**
   * Set the shooter to pass mode, which is a Constant velocity of about 20 RPM. This is for passing
   * the ball.
   */
  public default void pass() {
    _masterMotor.setControl(velocityRequest.withVelocity(PASS_RPS));
    targetVelocity = PASS_RPS;
  }

  public default void shoot() {
    _masterMotor.setControl(velocityRequest.withVelocity(shotVelocitySupplier.getAsDouble()));
    targetVelocity = shotVelocitySupplier.getAsDouble();
  }

  public default void stopMotor() {
    _masterMotor.stopMotor(); // TODO: figure out if we want to stop the motor or slow the motor
    targetVelocity = 0;
  }

  public default double getWantedVelocity() {
    return targetVelocity;
  }

  public default double caculateWantedSpeed() {
    /*
     * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡤⠖⠉⣉⣉⣉⡉⠉⠉⠒⠢⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
     * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡔⢋⣠⣶⠿⣛⡯⠭⠿⢟⣳⠶⣄⠠⠝⢆⠀⠀⠀⠀⠀⠀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
     * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡴⠉⣰⣾⢏⠵⠋⠁⠀⠀⠀⠀⠈⠙⢮⡢⢸⣮⢣⣀⠴⢒⣯⣍⡉⠉⠲⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀
     * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⡞⠁⣼⡿⣱⠋⠀⠀⠀⠀⠀⠀⣀⡤⠤⠖⣻⣿⣿⣿⣿⣾⣿⣿⣏⣛⡛⠦⣜⡂⠀⠀⠀⠀⠀⠀⠀⠀
     * ⠀⠀⠀⠀⠀⠀⠀⢀⡖⣫⣿⠁⣼⣿⢱⠋⠀⠀⠀⠀⠀⡴⠋⢰⣶⡿⣿⣿⣿⣿⣿⢿⡿⣿⣿⣿⣿⣿⣿⣶⣝⠢⡀⠀⠀⠀⠀⠀⠀
     * ⠀⠀⠀⠀⠀⠀⠀⠈⠉⢻⣿⡆⣿⣏⡏⠀⠀⠀⠀⠀⣸⠃⣰⢏⣵⣿⣿⣻⢯⡷⣯⣟⣿⣷⡿⣿⣿⣿⣿⣿⣿⣷⡱⡄⠀⠀⠀⠀⠀
     * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡏⠅⢺⣷⡇⠀⠀⠀⠀⠀⣿⣠⢯⢿⣿⣟⡾⣽⢯⣟⣷⣻⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢳⠀⠀⠀⠀⠀
     * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠁⢺⣿⢳⠀⠀⠀⠀⠀⠉⠁⢸⢺⣿⢾⣽⢯⣿⡽⣾⢷⣿⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣏⢇⠀⠀⠀⠀
     * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠀⢹⣿⡼⢗⣛⣏⣷⡄⠀⠀⢸⣹⣿⣯⣿⣿⢾⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣎⢦⡀⠀⠀
     * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡷⡇⠀⢡⣰⡾⣫⠞⠋⠁⠀⠀⠀⠙⣿⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀
     * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣧⢇⠀⣤⣿⢿⠁⠀⠀⠀⠀⠀⠀⣰⣻⣿⣳⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠀⠀
     * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢿⣹⠀⠸⣿⡞⡇⠀⠀⠀⠀⠀⠀⣿⣿⣳⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡀⠀⠀
     * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⣟⡇⠀⢿⣷⣸⡀⠀⠀⠀⠀⣰⢟⣯⢿⡾⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⡇⠀⠀
     * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣷⢻⠀⡘⣿⣇⢧⠀⠀⠀⠀⢷⡯⣟⣯⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⡿⠿⠛⠉⠀⠀⠀
     * ⠀⠀⠀⠀⠀⢀⣀⣀⣀⠀⠀⠸⡽⡇⠠⢹⣿⡌⣇⠀⠀⠀⠀⠈⠉⠉⠉⠙⠿⣻⣿⠟⢯⣿⣽⠿⢿⣿⡿⠿⠚⠀⠀⠀⠀⠀⠀⠀⠀
     * ⠀⠀⣠⠴⠊⠙⢿⢳⣫⢍⡲⢴⣷⢿⡄⠡⢿⣿⡘⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠉⠉⠉⠉⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
     * ⢀⡼⢁⠤⡠⠀⠀⠓⣌⠓⢝⠶⣽⣛⠎⡀⢭⣿⣷⡹⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
     * ⢸⠰⣏⢲⣅⢃⠀⣀⠈⢧⡀⠱⣝⢿⣿⣶⣹⢾⣿⣧⢹⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡰⣲⡲⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
     * ⣸⢸⣯⢖⣯⢾⡽⣶⣻⡎⡇⠀⠈⢳⣻⣿⣳⢿⣿⣿⣇⢳⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡸⣿⣿⣗⣇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
     * ⢾⣨⣿⡜⣯⣿⢿⣿⣿⡇⣷⠀⠀⠈⢧⢻⣯⣿⣿⣿⣿⡎⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⡜⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀
     * ⠈⢳⡹⣷⡹⣾⣿⣿⣿⣧⣿⠀⠀⠀⠘⡟⣿⣿⣯⣿⣿⣷⢻⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠐⣿⣿⣿⣿⣿⣞⢆⠀⠀⠀⠀⠀⠀⠀⠀
     * ⠀⠀⠱⡝⣿⡽⣿⣿⢏⡜⠁⠀⠀⠀⠀⢽⢹⣿⣿⣿⣿⣿⠼⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢟⣿⣿⣿⣿⣿⣧⣕⠦⣀⠀⠀⠀⠀⠀
     * ⠀⠀⠀⠀⢸⢽⡿⢣⠎⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⡃⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⢿⢿⣿⣻⡿⣿⡿⣿⣾⣿⣯⣖⣒⡀
     * ⠀⠀⠀⠀⠘⣪⠖⠁⠀⠀⠀⠀⠀⠀⠀⢸⢷⣿⣿⣿⣿⣿⡁⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢿⣻⣽⣷⣿⣳⣯⣟⣷⣟⡿⠛
     * ⠀⠀⠀⠈⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⢸⡿⣽⣾⢿⣿⢀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠒⠒⠒⠋⠉⠀⠀⠀
     */
    return 0;
  }
}
