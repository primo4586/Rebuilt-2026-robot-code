package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.primoLib.PrimoCalc;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederSim;
import frc.robot.subsystems.feeder.FeederTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodSim;
import frc.robot.subsystems.hood.HoodTalon;
import frc.robot.subsystems.intake.intakeArm.IntakeArm;
import frc.robot.subsystems.intake.intakeArm.IntakeArmSim;
import frc.robot.subsystems.intake.intakeArm.IntakeArmTalon;
import frc.robot.subsystems.intake.intakeRoller.IntakeRoller;
import frc.robot.subsystems.intake.intakeRoller.IntakeRollerSim;
import frc.robot.subsystems.intake.intakeRoller.IntakeRollerTalon;
import frc.robot.subsystems.shootOnTheMove.ShotCalculator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterRealIO;
import frc.robot.subsystems.shooter.ShooterSimIO;
import java.util.function.DoubleSupplier;

public class CommandGroupFactory {
  private static final Drive drive = Drive.getInstance(RobotBase.isReal());
  private static final Shooter shooter =
      Shooter.getInstance(RobotBase.isReal() ? new ShooterRealIO() : new ShooterSimIO());
  private static final Hood hood =
      Hood.getInstance(RobotBase.isReal() ? new HoodTalon() : new HoodSim());
  public static final Feeder feeder =
      Feeder.getInstance(RobotBase.isReal() ? new FeederTalonFX() : new FeederSim());
  public static final IntakeArm intakeArm =
    IntakeArm.getInstance(RobotBase.isReal() ? new IntakeArmTalon() : new IntakeArmSim());
  public static final IntakeRoller intakeRoller =
    IntakeRoller.getInstance(RobotBase.isReal() ? new IntakeRollerTalon() : new IntakeRollerSim());
  public static final ShotCalculator shotCalculator = ShotCalculator.getInstance();

  public static final DoubleSupplier distanceSotmSupplier = () -> drive.getPose().getTranslation()
    .getDistance(shotCalculator.getCurrentEffectiveTargetPose().getTranslation().toTranslation2d());


  /** turn to hub, stop with x, and shoot with interpolation */
  public static Command shootCommand() {
    return Commands.sequence(
            Commands.deadline(
                Commands.sequence(
                    Commands.waitSeconds(0.02),
                    Commands.waitUntil(
                        () ->
                            PrimoCalc.isFacingHub().getAsBoolean()
                                && shooter.readyToShoot().getAsBoolean())),
                hood.setPositionWithInterpolation(),
                shooter.shoot(),
                DriveCommands.joystickDriveAtAngle(
                    drive, () -> 0.0, () -> 0.0, () -> new Rotation2d(PrimoCalc.getRadsToHub()))),
            Commands.parallel(feeder.feed(), Commands.run(() -> drive.stopWithX())))
        .finallyDo(() -> shooter.rest());
  }

    /** Shoot on the move*/
    public static Command shootOnTheMoveCommand(Command driveCommand) {
      return Commands.parallel(
        targetHubSotmCommand(),
        feeder.feed(),
        driveCommand
      ).finallyDo(() -> shooter.rest());
    }

    /**
     * @return a command that stops all subsystems (intake arm, intake roller, shooter, feeder)
     */
  public static Command stopAll(){
    return Commands.parallel(
        intakeArm.setVoltage(0), 
        intakeRoller.setVoltage(0), 
        shooter.restCommand(),
        feeder.setVoltage(0));
  }

  /**
   * @return a command that sets shooter velocity and hood position based on distance to target for shoot on the move
   */
  public static Command targetHubSotmCommand() {
    return Commands.parallel(
            shooter.setVelocityCommand(
                    () -> ShooterConstants.SHOOTER_INTERPOLATION_MAP.get(distanceSotmSupplier.getAsDouble())).asProxy(),
            hood.setPositionRepeatedly(
                    () -> HoodConstants.HOOD_ANGLE_INTERPOLATION_MAP.get(distanceSotmSupplier.getAsDouble())).asProxy())
            .andThen(useRequirement());
}

private static Command useRequirement() {
  return Commands.runOnce(() -> {
  });
}

  /**
   * @return Command that shoots + feed + hood interpolation
   */
  public static Command instantShoot(){
    return Commands.parallel(shooter.shoot(), feeder.feed(), hood.setPositionWithInterpolation());
  }

  /**
   * returns System.out.println as a command
   *
   * @return System.out.println as a command
   * @param s string to print
   */
  public static Command printCommand(String output) {
    return Commands.runOnce(() -> System.out.println(output));
  }

  /**
   * returns System.out.println as a command
   *
   * @return System.out.println as a command
   * @param d double to print
   */
  public static Command printCommand(DoubleSupplier output) {
    return Commands.runOnce(() -> System.out.println(output.getAsDouble()));
  }
}
