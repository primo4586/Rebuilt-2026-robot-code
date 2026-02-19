// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterRealIO;
import frc.robot.subsystems.shooter.ShooterSimIO;
import frc.robot.subsystems.intake.intakeArm.IntakeArm;
import frc.robot.subsystems.intake.intakeArm.IntakeArmIO;
import frc.robot.subsystems.intake.intakeArm.IntakeArmSim;
import frc.robot.subsystems.intake.intakeArm.IntakeArmTalon;
import frc.robot.subsystems.intake.intakeRoller.IntakeRoller;
import frc.robot.subsystems.intake.intakeRoller.IntakeRollerIO;
import frc.robot.subsystems.intake.intakeRoller.IntakeRollerSim;
import frc.robot.subsystems.intake.intakeRoller.IntakeRollerTalon;
import frc.robot.subsystems.vision.*;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Vision vision;
  private final Drive drive;
  private final Shooter shooter;
  private final IntakeRoller intakeRoller;
  private final IntakeArm intakeArm;

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
        shooter = new Shooter(new ShooterRealIO());
        intakeRoller = IntakeRoller.getInstance(new IntakeRollerTalon());
        intakeArm = IntakeArm.getInstance(new IntakeArmTalon());
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
        shooter = new Shooter(new ShooterSimIO());
        intakeRoller = IntakeRoller.getInstance(new IntakeRollerSim());
        intakeArm = IntakeArm.getInstance(new IntakeArmSim());
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
        shooter = new Shooter(new ShooterIO() {});
        intakeRoller = IntakeRoller.getInstance(new IntakeRollerIO() {});
        intakeArm = IntakeArm.getInstance(new IntakeArmIO() {});
        break;
    }

    // named commands

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

    driveController.a().whileTrue(intakeRoller.setVoltage(6));
    driveController.b().whileTrue(intakeRoller.setCurrent(-400));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
                drive,
                () -> -driveController.getLeftY() * slowSpeed.getAsDouble(),
                () -> -driveController.getLeftX() * slowSpeed.getAsDouble(),
                () -> -driveController.getRightX() * slowSpeed.getAsDouble())
            .withName("Drive"));

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
    testerController.a().whileTrue(shooter.passCommand());
    testerController.b().whileTrue(shooter.setVoltageCommand(6));
  }

  /**
   * Use this to pass the autonomous command t o the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void periodic() {}
}
