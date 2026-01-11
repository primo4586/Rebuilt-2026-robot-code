package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.commands.CommandGroupFactory;
import frc.robot.subsystems.Cannon.CannonRealIO;
import frc.robot.subsystems.Cannon.CannonSimIO;
import frc.robot.subsystems.Cannon.CannonSubsystem;
import frc.robot.subsystems.disposer.Disposer;
import frc.robot.subsystems.disposer.DisposerReal;
import frc.robot.subsystems.disposer.DisposerSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class RobotState {
  // subsystems
  private static CannonSubsystem cannon =
      CannonSubsystem.getInstance(RobotBase.isReal() ? new CannonRealIO() : new CannonSimIO());
  private static Elevator elevator =
      Elevator.getInstance(RobotBase.isReal() ? new ElevatorIOReal() : new ElevatorIOSim());
  private static Disposer disposer =
      Disposer.getInstance(RobotBase.isReal() ? new DisposerReal() : new DisposerSim());

  // is ready functions
  public static BooleanSupplier isElevatorReady = () -> elevator.isElevatorReady();
  public static BooleanSupplier isDispserReady = () -> disposer.isReady();
  /*
   * 0 - driving to home
   * 1 - align home
   * 2 - driving to reef
   * 3 - align reef
   * 4 - scoring
   * 5 - driver control
   */
  public static IntSupplier drivingState = () -> 0;
  // TODO fix meserments
  public static IntSupplier side = () -> 1;
  public static DoubleSupplier wanted_x = () -> Constants.sides[side.getAsInt() - 1].getX();
  public static DoubleSupplier wanted_y = () -> Constants.sides[side.getAsInt() - 1].getY();
  public static DoubleSupplier wanted_angle =
      () -> Constants.sides[side.getAsInt() - 1].getRotation().getDegrees();

  public static boolean rightHome = true;
  public static DoubleSupplier Home_x = () -> rightHome ? 2.5 : 2.5; // TODO: constatnts
  public static DoubleSupplier Home_y = () -> rightHome ? 1.684 : 6.8;
  public static DoubleSupplier Home_angle = () -> rightHome ? 50 : -50.363;

  public static BooleanSupplier algeaOut = () -> true;
  public static BooleanSupplier rightReef = () -> true;

  public static DoubleSupplier elevatorHeight = () -> ElevatorConstants.L4_HEIGHT;
  public static DoubleSupplier algeaHeight =
      () -> CommandGroupFactory.getAlgeaHeight(side.getAsInt());
}
