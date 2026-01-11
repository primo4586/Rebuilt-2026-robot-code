package frc.robot.commands;

import static frc.robot.Constants.*;
import static frc.robot.RobotState.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.Cannon.CannonSubsystem;
import frc.robot.subsystems.disposer.Disposer;
import frc.robot.subsystems.disposer.DisposerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import java.io.IOException;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.json.simple.parser.ParseException;

public class CommandGroupFactory {

  private final CannonSubsystem cannon;
  private final Disposer disposer;
  private final Elevator elevator;
  private final Drive drive;

  public CommandGroupFactory(
      CannonSubsystem cannon, Disposer disposer, Elevator elevator, Drive drive) {
    this.cannon = cannon;
    this.disposer = disposer;
    this.elevator = elevator;
    this.drive = drive;
  }

  /**
   * returns a command that pathfinds to reef and places a coral coral level is determined by the
   * elevatorHeight var in robotcontainer
   *
   * @param side reef side (1: side that points towards driver station, goes up to 6 counter
   *     clockwise)
   * @param isLeft determines if the robot will go to the left point of the reef side
   * @return command that pathfinds to reef and places a coral
   */
  // public Command pathfindToReefCoralCommand(int side, BooleanSupplier isLeft) {
  // try {
  // return Commands.sequence(
  // loadPath(side, isLeft),
  // cannon.shootAndStopCommand(), // TODO: add elevator height check
  // Commands.waitSeconds(0.5), // TODO: tune
  // elevator.resetCommand());

  // } catch (FileVersionException | ParseException | IOException e) {
  // e.printStackTrace();
  // // Report to DS so you see it during a match
  // edu.wpi.first.wpilibj.DriverStation.reportError(
  // "Failed to load path file: " + choosePointPath(side, isLeft),
  // e.getStackTrace());

  // // Return a "do nothing" command instead of crashing robot code
  // return edu.wpi.first.wpilibj2.command.Commands.none();
  // }
  // }

  /**
   * returns a command that pathfinds to reef, removes algea and places a coral always on the right
   * point of the reef side so algea can be removed
   *
   * @param side reef side (1: side that points towards driver station, goes up to 6 counter
   *     clockwise)
   * @param coralHeight height to place coral at in rotations
   * @return command that pathfinds to reef, removes algea and places a coral
   */
  // public Command pathfindToReefAlgeaCommand(int side, double coralHeight) {
  // try {
  // return Commands.sequence(loadPath(side, () -> false)).withName("alfea
  // pathfinding");
  // } catch (FileVersionException | ParseException | IOException e) {
  // e.printStackTrace();
  // // Report to DS so you see it during a match
  // edu.wpi.first.wpilibj.DriverStation.reportError(
  // "Failed to load path file: " + choosePointPath(side, () -> false),
  // e.getStackTrace());
  // // Return a "do nothing" command instead of crashing robot code
  // return Commands.none();
  // }
  // }

  /**
   * returns the reef point path name as a string
   *
   * @param side reef side (1: side that points towards driver station, goes up to 6 counter
   *     clockwise)
   * @param isLeft determines if the robot will go to the left point of the reef side
   * @return reef point path name
   * @throws ParseException
   * @throws IOException
   * @throws FileVersionException
   */
  private PathPlannerPath choosePointPath()
      throws FileVersionException, IOException, ParseException {
    String[] sides = {"A", "B", "C", "D", "E", "F"};
    String point = rightReef.getAsBoolean() ? "R" : "L";
    String pathName = sides[side.getAsInt() - 1] + point; // e.g. "LeftAuto"
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    return path;
  }

  public Command manualScoreCommand() {
    try {
      return Commands.sequence(alignToReef(), Score(), alignToHome());
      // TODO: bring back old code for choosing the closest reef
    } catch (FileVersionException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    } catch (ParseException e) {
      e.printStackTrace();
    }
    return Commands.none();
  }

  /**
   * returns algea height depending on the reef side
   *
   * @param side reef side (1: side that points towards driver station, goes up to 6 counter
   *     clockwise)
   * @return algea height in rotations
   */
  public static double getAlgeaHeight(int side) {
    if (side == 1 || side == 3 || side == 5) return 6;
    return ElevatorConstants.L2_HEIGHT;
  }

  /**
   * returns a command that pathfinds to and follows a path
   *
   * @param side reef side (1: side that points towards driver station, goes up to 6 counter
   *     clockwise)
   * @param isLeft determines if the robot will go to the left point of the reef side
   * @return a command that pathfinds to and follows a path
   */
  // public Command loadPath(int side, BooleanSupplier isLeft)
  // throws FileVersionException, ParseException, IOException {
  // PathPlannerPath path;
  // String path_string = choosePointPath(side, isLeft);
  // path = PathPlannerPath.fromPathFile(path_string);
  // return AutoBuilder.pathfindThenFollowPath( // pathfinding command
  // path,
  // new PathConstraints(
  // PATHFINDER_MAX_VELOCITY,
  // PATHFINDER_MAX_ACCELRATION,
  // PATHFINDER_MAX_ANGULAR_VELOCITY,
  // PATHFINDER_MAX_ANGULAR_ACCELARATION));
  // }

  public Command driveToReef() {
    return Commands.sequence(
        Commands.runOnce(() -> drivingState = () -> 2),
        driveToPostion(wanted_x, wanted_y, wanted_angle));
  }

  public Command driveToHome() {
    return Commands.sequence(
        elevator.relocateCommand(ElevatorConstants.AUTO_HEIGHT),
        Commands.runOnce(() -> drivingState = () -> 0),
        driveToPostion(Home_x, Home_y, Home_angle));
  }

  public Command alignToReef() throws FileVersionException, IOException, ParseException {
    return Commands.sequence(
        Commands.runOnce(() -> drivingState = () -> 3), AutoBuilder.followPath(choosePointPath()));
  }

  public Command alignToHome() throws FileVersionException, IOException, ParseException {
    return Commands.sequence(
        Commands.runOnce(() -> drivingState = () -> 1),
        rightHome
            ? AutoBuilder.followPath(PathPlannerPath.fromPathFile("rightHome"))
            : AutoBuilder.followPath(PathPlannerPath.fromPathFile("leftHome")));
  }

  public Command switchSidesCommand(int wantedSide) {
    return Commands.defer(
        () ->
            Commands.sequence(
                Commands.runOnce(() -> side = () -> wantedSide),
                Commands.either(
                    new PathPlannerAuto("Test"),
                    Commands.none(),
                    () -> drivingState.getAsInt() == 2)),
        Set.of());
  }

  // public Command switchIntakeCommand() {
  //   return Commands.defer(
  //       () -> {
  //         try {
  //           return Commands.sequence(
  //               Commands.runOnce(() -> rightHome = !rightHome),
  //               Commands.either(
  //                   Commands.sequence(driveToHome(), alignToHome()),
  //                   Commands.none(),
  //                   () -> drivingState.getAsInt() == 0));
  //         } catch (FileVersionException e) {
  //           e.printStackTrace();
  //         } catch (IOException e) {
  //           e.printStackTrace();
  //         } catch (ParseException e) {
  //           e.printStackTrace();
  //         }
  //         return Commands.none();
  //       },
  //       Set.of());
  // }

  public Command driveToPostion(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angle) {
    return AutoBuilder.pathfindToPoseFlipped(
        new Pose2d(x.getAsDouble(), y.getAsDouble(), Rotation2d.fromDegrees(angle.getAsDouble())),
        new PathConstraints(
            PATHFINDER_MAX_VELOCITY,
            PATHFINDER_MAX_ACCELRATION,
            PATHFINDER_MAX_ANGULAR_VELOCITY,
            PATHFINDER_MAX_ANGULAR_ACCELARATION));
  }

  public Command Score() {
    return Commands.sequence(
            // pepration sequence
            Commands.runOnce(() -> drivingState = () -> 4),
            disposer.moveToPositionCommand(DisposerConstants.HOME_POSITION),
            elevator.setHeightScoreCommand(),
            Commands.waitSeconds(2),
            Commands.waitUntil(isElevatorReady),
            cannon.shootAndStopCommand(),
            elevator.relocateCommand(0)) // scoring
        .withName("Score")
        .finallyDo(() -> elevator.relocate(0));
  }

  public Command removeAlgea() {
    try {
      return Commands.sequence(
              driveToReef(),
              elevator.setHeightAlgeaCommand(),
              disposer.moveToPositionCommand(DisposerConstants.OPEN_POSITION),
              alignToReef(),
              disposer.moveToPositionCommand(0.5),
              Commands.waitSeconds(0.5),
              Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-1, 0, 0)), drive)
                  .withTimeout(0.7),
              elevator.relocateCommand(0),
              disposer.moveToPositionCommand(0),
              driveToHome())
          .finallyDo(() -> elevator.relocate(0));
    } catch (FileVersionException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    return Commands.none();
  }

  /**
   * Return reef side id 1..6 that the robot is closest to. Clockwise ordering: AB=1, KL=2, IJ=3,
   * GH=4, EF=5, CD=6
   */
  public int getClosestReefSide() {
    Pose2d cur = drive.getPose();
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
    Translation2d center = isBlue ? new Translation2d(4.5, 4) : new Translation2d(13, 4);

    // vector from robot -> reef center
    double dx = center.getX() - cur.getX();
    double dy = center.getY() - cur.getY();

    // angle in degrees, -90..+90
    double angleDeg = Math.toDegrees(Math.atan(dy / dx));

    // original angle-based zones (using atan’s natural range)
    // (-90, -30] → zone 0
    // (-30,  30] → zone 1
    // else        → zone 2
    int zone;
    if (angleDeg > -90.0 && angleDeg <= -30.0) {
      zone = 0;
    } else if (angleDeg > -30.0 && angleDeg <= 30.0) {
      zone = 1;
    } else {
      zone = 2;
    }

    boolean rightOfCenter = cur.getX() > center.getX();

    final int[][] blue = {
      {2, 5}, // zone0: left->KL(2), right->EF(5)
      {1, 4}, // zone1: left->AB(1), right->GH(4)
      {6, 3} // zone2: left->CD(6), right->IJ(3)
    };
    final int[][] red = {
      {5, 2}, // zone0: left->EF(5), right->KL(2)
      {4, 1}, // zone1: left->GH(4), right->AB(1)
      {3, 6} // zone2: left->IJ(3), right->CD(6)
    };

    // optional debug
    // System.out.printf(Locale.US,
    //     "cur=(%.2f, %.2f) center=(%.2f, %.2f) dx=%.2f dy=%.2f angle=%.2f zone=%d right=%b
    // isBlue=%b%n",
    //     cur.getX(), cur.getY(), center.getX(), center.getY(), dx, dy, angleDeg, zone,
    // rightOfCenter, isBlue);

    return (isBlue ? blue : red)[zone][rightOfCenter ? 1 : 0];
  }

  /**
   * returns System.out.println as a command
   *
   * @return System.out.println as a command
   * @param s string to print
   */
  public Command printCommand(String s) {
    return Commands.runOnce(() -> System.out.println(s));
  }

  public Command newLeftAuto() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              RobotState.rightReef = () -> true;
              RobotState.side = () -> 3;
              RobotState.elevatorHeight = () -> ElevatorConstants.L4_HEIGHT;
              RobotState.rightHome = false;
            }),
        new PathPlannerAuto("AutoCoral"),
        Commands.runOnce(
            () -> {
              RobotState.side = () -> 2;
              RobotState.rightReef = () -> false;
              RobotState.elevatorHeight = () -> ElevatorConstants.L4_HEIGHT;
            }),
        new PathPlannerAuto("AutoCoral"),
        Commands.runOnce(
            () -> {
              RobotState.side = () -> 2;
              RobotState.rightReef = () -> true;
              RobotState.elevatorHeight = () -> ElevatorConstants.L4_HEIGHT;
            }),
        new PathPlannerAuto("AutoCoral"));
  }

  public Command newRightAuto() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              RobotState.rightReef = () -> false;
              RobotState.side = () -> 5;
              RobotState.elevatorHeight = () -> ElevatorConstants.L4_HEIGHT;
              RobotState.rightHome = true;
            }),
        new PathPlannerAuto("AutoCoral"),
        Commands.runOnce(
            () -> {
              RobotState.side = () -> 6;
              RobotState.rightReef = () -> false;
              RobotState.elevatorHeight = () -> ElevatorConstants.L4_HEIGHT;
            }),
        new PathPlannerAuto("AutoCoral"),
        Commands.runOnce(
            () -> {
              RobotState.side = () -> 6;
              RobotState.rightReef = () -> true;
              RobotState.elevatorHeight = () -> ElevatorConstants.L4_HEIGHT;
            }),
        new PathPlannerAuto("AutoCoral"));
  }

  public Command newMiddleAuto() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              RobotState.rightReef = () -> false;
              RobotState.side = () -> 4;
              RobotState.elevatorHeight = () -> ElevatorConstants.L4_HEIGHT;
            }),
        new PathPlannerAuto("AutoCoral NO FEEDER"));
  }
}
