// package frc.robot.subsystems.shooter;

// import static frc.robot.util.SparkUtil.*;

// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.simulation.BatterySim;
// import edu.wpi.first.wpilibj.simulation.RoboRioSim;

// public class ShooterSim implements ShooterIO {

//   public CannonSimIO() {
//     // configure motor
//     configs.smartCurrentLimit(MAX_CURRENT).idleMode(IDLE_MODE).inverted(INVERTED);

//     tryUntilOk(
//         motor,
//         5,
//         () ->
//             motor.configure(
//                 configs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
//   }

//   @Override
//   public void updateInputs(CannonIOInputs inputs) {
//     /*
//      * I mean, I don't know, it works.
//      * If there is a time problem, just delete the sim, we don't even use the speed.
//      * The only thing that I use is the current.
//      */
//     // get motor sim deta
//     motorSim.setBusVoltage(RobotController.getBatteryVoltage());
//     motorSim.setAppliedOutput(motor.get());

//     // update sim
//     sim.setInput(motorSim.getAppliedOutput() * motorSim.getBusVoltage());
//     sim.update(Timer.getFPGATimestamp());
//     RoboRioSim.setVInVoltage(
//         BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

//     // log inputs
//     ifOk(motor, voltageSupplier, (value) -> inputs.voltage = value);
//     ifOk(motor, currentSupplier, (value) -> inputs.current = value);
//     inputs.opticalSensor = getSensor();
//   }
// }
