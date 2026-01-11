// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.RobotState.elevatorHeight;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.CommandGroupFactory;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Cannon.CannonIO;
import frc.robot.subsystems.Cannon.CannonRealIO;
import frc.robot.subsystems.Cannon.CannonSimIO;
import frc.robot.subsystems.Cannon.CannonSubsystem;
import frc.robot.subsystems.disposer.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.vision.*;
import java.io.IOException;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final CannonSubsystem cannon;
  private final Disposer disposer;
  private final Elevator elevator;

  // command group factory
  private final CommandGroupFactory commandGroupFactory;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController testerController = new CommandXboxController(1);
  private final CustomController operatorController = new CustomController(3);
  // SlowMode
  // private final DoubleSupplier slowSpeed =
  //     () -> driveController.leftTrigger().getAsBoolean() ? 0.5 : 0.8;
  private final DoubleSupplier slowSpeed =
      () -> driveController.leftBumper().getAsBoolean() ? 0.5 : 0.8;

  // operator pathplanner values
  // TODO: move to robot state

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(cameraOPI, cameraOPITranslation),
                new VisionIOPhotonVision(cameraElevator, cameraElevatorTranslation));
        cannon = CannonSubsystem.getInstance(new CannonRealIO());
        disposer = Disposer.getInstance(new DisposerReal());
        elevator = Elevator.getInstance(new ElevatorIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(cameraOPI, cameraOPITranslation, drive::getPose),
                new VisionIOPhotonVisionSim(
                    cameraElevator, cameraElevatorTranslation, drive::getPose));
        cannon = CannonSubsystem.getInstance(new CannonSimIO());
        disposer = Disposer.getInstance(new DisposerSim());
        elevator = Elevator.getInstance(new ElevatorIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations0
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        cannon = CannonSubsystem.getInstance(new CannonIO() {});
        disposer = Disposer.getInstance(new DisposerIO() {});
        elevator = Elevator.getInstance(new ElevatorIO() {});
        break;
    }

    // command group factory
    commandGroupFactory = new CommandGroupFactory(cannon, disposer, elevator, drive);

    // named commands
    @SuppressWarnings("rawtypes")
    Set systemSet = Set.of(cannon, disposer, elevator);
    @SuppressWarnings("rawtypes")
    Set swerveSet = Set.of(drive);
    Set allSet = Set.of(drive, disposer, elevator, cannon);
    NamedCommands.registerCommand(
        "align to reef",
        Commands.defer(
            () -> {
              try {
                return commandGroupFactory.alignToReef();
              } catch (FileVersionException | IOException | ParseException e) {
                e.printStackTrace();
              }
              return Commands.none();
            },
            swerveSet));

    NamedCommands.registerCommand(
        "align to home",
        Commands.defer(
            () -> {
              try {
                return commandGroupFactory.alignToHome();
              } catch (FileVersionException | IOException | ParseException e) {
                e.printStackTrace();
              }
              return Commands.none();
            },
            swerveSet));
    NamedCommands.registerCommand(
        "drive to reef", Commands.defer(() -> commandGroupFactory.driveToReef(), swerveSet));
    NamedCommands.registerCommand(
        "drive to home", Commands.defer(() -> commandGroupFactory.driveToHome(), swerveSet));
    NamedCommands.registerCommand(
        "score", Commands.defer(() -> commandGroupFactory.Score(), systemSet));
    NamedCommands.registerCommand(
        "remove algea", Commands.defer(() -> commandGroupFactory.removeAlgea(), allSet));
    NamedCommands.registerCommand("L1", elevator.relocateCommand(ElevatorConstants.L1_HEIGHT));
    NamedCommands.registerCommand("L2", elevator.relocateCommand(ElevatorConstants.L2_HEIGHT));
    NamedCommands.registerCommand("L3", elevator.relocateCommand(ElevatorConstants.L3_HEIGHT));
    NamedCommands.registerCommand("L4", elevator.relocateCommand(ElevatorConstants.L4_HEIGHT));
    NamedCommands.registerCommand("Edge", cannon.edgeTimeCommand());
    NamedCommands.registerCommand("shootAndStop", cannon.edgeShootCommand());
    NamedCommands.registerCommand("LAuto", elevator.relocateCommand(ElevatorConstants.AUTO_HEIGHT));
    NamedCommands.registerCommand("waitForCoral", cannon.waitForCoral());
    NamedCommands.registerCommand(
        "isElevatorReady", Commands.waitUntil(() -> elevator.isElevatorReady()));
    NamedCommands.registerCommand("new Left", commandGroupFactory.newLeftAuto());
    NamedCommands.registerCommand("new Right", commandGroupFactory.newRightAuto());
    NamedCommands.registerCommand("new Middle", commandGroupFactory.newMiddleAuto());

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // event triggers
    new EventTrigger("L1").onTrue(elevator.relocateCommand(ElevatorConstants.L1_HEIGHT));
    new EventTrigger("L2").onTrue(elevator.relocateCommand(ElevatorConstants.L2_HEIGHT));
    new EventTrigger("L3").onTrue(elevator.relocateCommand(ElevatorConstants.L3_HEIGHT));
    new EventTrigger("L4").onTrue(elevator.relocateCommand(ElevatorConstants.L4_HEIGHT));
    new EventTrigger("LAuto").onTrue(elevator.relocateCommand(ElevatorConstants.AUTO_HEIGHT));

    // coral edge trigger
    // CannonConstants.CORAL_EDGE_TRIGGER.onTrue(cannon.edgeCommand());

    // Configure the button bindings

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
                drive,
                () -> -driveController.getLeftY() * slowSpeed.getAsDouble(),
                () -> -driveController.getLeftX() * slowSpeed.getAsDouble(),
                () -> -driveController.getRightX() * slowSpeed.getAsDouble())
            .withName("Drive"));

    // testerController.leftTrigger().whileTrue(drive);

    // controller.a().whileTrue(cannon.setVoltageCommand(3));
    // controller.b().onTrue(disposer.moveToPositionCommand(DisposerConstants.HOME_POSITION));
    // controller.x().onTrue(disposer.moveToPositionCommand(DisposerConstants.OPEN_POSITION));
    // controller.y().whileTrue(elevator.setVoltageCommand(1));

    // controller.a().onTrue(Commands.runOnce(() -> isLeft = false));
    // controller.b().onTrue(Commands.runOnce(() -> isLeft = false));
    // controller.y().onTrue(new PathPlannerAuto("Test"));
    // driveController.a().onTrue(elevator.relocateCommand(0));
    // driveController.y().onTrue(elevator.relocateCommand(5));
    // driveController.b().onTrue(elevator.relocateCommand(10));
    // driveController.x().onTrue(cannon.edgeCommand());
    operatorController.button(2).onTrue(commandGroupFactory.switchSidesCommand(1));
    operatorController.button(1).onTrue(commandGroupFactory.switchSidesCommand(2));
    operatorController.button(7).onTrue(commandGroupFactory.switchSidesCommand(3));
    operatorController.button(8).onTrue(commandGroupFactory.switchSidesCommand(4));
    operatorController.button(9).onTrue(commandGroupFactory.switchSidesCommand(5));
    operatorController.button(3).onTrue(commandGroupFactory.switchSidesCommand(6));
    operatorController
        .button(10)
        .onTrue(Commands.runOnce(() -> elevatorHeight = () -> ElevatorConstants.L2_HEIGHT));
    operatorController
        .button(11)
        .onTrue(Commands.runOnce(() -> elevatorHeight = () -> ElevatorConstants.L3_HEIGHT));
    operatorController
        .button(12)
        .onTrue(Commands.runOnce(() -> elevatorHeight = () -> ElevatorConstants.L4_HEIGHT));
    operatorController.button(4).onTrue(changeSidesLeft());
    operatorController.button(5).onTrue(changeSidesRightAlgea());
    operatorController.button(6).onTrue(Commands.runOnce(() -> RobotState.rightReef = () -> true));
    operatorController
        .button(14)
        .onTrue(Commands.runOnce(() -> RobotState.rightHome = !RobotState.rightHome));
    operatorController.button(15).whileFalse(new PathPlannerAuto("Test").repeatedly());

    testerController.a().onTrue(elevator.relocateCommand(ElevatorConstants.L1_HEIGHT));
    testerController.b().onTrue(elevator.relocateCommand(ElevatorConstants.L2_HEIGHT));
    testerController.x().onTrue(elevator.relocateCommand(ElevatorConstants.L3_HEIGHT));
    testerController.y().onTrue(elevator.relocateCommand(ElevatorConstants.L4_HEIGHT));
    testerController.rightBumper().whileTrue(elevator.setVoltageCommand(2));
    testerController.leftBumper().whileTrue(elevator.setVoltageCommand(-2));
    testerController.back().onTrue(cannon.shootAndStopCommand());
    testerController.start().onTrue(elevator.setPositionCommand(0));
    driveController.rightTrigger().whileTrue(new PathPlannerAuto("AutoAlgea"));
    driveController.leftTrigger().whileTrue(new PathPlannerAuto("AutoCoral"));
    driveController.leftStick().onTrue(cannon.edgeTimeCommand());
    driveController
        .a()
        .onTrue(
            Commands.runOnce(() -> RobotState.elevatorHeight = () -> ElevatorConstants.L4_HEIGHT));
    driveController
        .b()
        .onTrue(
            Commands.runOnce(() -> RobotState.elevatorHeight = () -> ElevatorConstants.L3_HEIGHT));
    driveController
        .x()
        .onTrue(
            Commands.runOnce(() -> RobotState.elevatorHeight = () -> ElevatorConstants.L2_HEIGHT));
    driveController.y().onTrue(commandGroupFactory.removeAlgea());
    driveController
        .start()
        .onTrue(
            Commands.parallel(
                disposer.moveToPositionCommand(DisposerConstants.HOME_POSITION),
                elevator.resetCommand()));
    testerController
        .rightTrigger()
        .whileTrue(
            Commands.run(
                () ->
                    drive.runVelocity(
                        new ChassisSpeeds(Units.inchesToMeters(1.8) * 200 * Math.PI, 0, 0)),
                drive));

    // goofy ahh drive shit
    // // Lock to 0° when A button is held
    // controller
    // .a()
    // .whileTrue(
    // DriveCommands.joystickDriveAtAngle(
    // drive,
    // () -> -controller.getLeftY(),
    // () -> -controller.getLeftX(),
    // () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driveController
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // try {
    //   driveController
    //       .leftBumper()
    //       .onTrue(AutoBuilder.followPath(PathPlannerPath.fromPathFile("rot")));
    // } catch (FileVersionException | IOException | ParseException e) {
    //   System.out.println("unalbe to follow given path!");
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
  }

  /**
   * Use this to pass the autonomous command t o the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Command changeSidesLeft() {
    return Commands.runOnce(
        () -> {
          RobotState.rightReef = () -> false;
          RobotState.algeaOut = () -> false;
        });
  }

  public Command changeSidesRightAlgea() {
    return Commands.runOnce(
        () -> {
          RobotState.rightReef = () -> true;
          RobotState.algeaOut = () -> true;
        });
  }

  public void periodic() {}
}
