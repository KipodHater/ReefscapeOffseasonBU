// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveStates;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorStates;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.gripper.*;
import frc.robot.subsystems.vision.*;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;
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
  private final Gripper gripper;
  private final Arm arm;
  private final Vision vision;
  private final Elevator elevator;
  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private DoubleSupplier m_controllerLeftX;
  private DoubleSupplier m_controllerLeftY;
  private DoubleSupplier m_controllerRightX;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                (robotPose) -> {},
                controller::getLeftX,
                controller::getLeftY,
                () -> controller.getRawAxis(4));
        gripper = new Gripper(new GripperIOSpark(), new GripperSensorIORev());
        arm = new Arm(new ArmIOSpark());
        elevator = new Elevator(new ElevatorIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO[] {
                  /*
                  new VisionIOPhoton("camera0", VisionConstants.robotToCamera0),
                  new VisionIOPhoton("camera1", VisionConstants.robotToCamera1),
                  new VisionIOLimelight("limelight-tsachi", RobotState.getInstance()::getYaw)*/
                });
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                (robotPose) -> driveSimulation.getSimulatedDriveTrainPose(),
                controller::getLeftX,
                controller::getLeftY,
                () -> controller.getRawAxis(2));

        gripper = new Gripper(new GripperIOSim(driveSimulation), new GripperSensorIO() {});
        arm = new Arm(new ArmIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO[] {
                  new VisionIOPhotonSim(
                      "camera0",
                      VisionConstants.robotToCamera0,
                      driveSimulation::getSimulatedDriveTrainPose),
                  new VisionIOPhotonSim(
                      "camera1",
                      VisionConstants.robotToCamera1,
                      driveSimulation::getSimulatedDriveTrainPose)
                  // new VisionIOTest()
                });
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (robotPose) -> {},
                controller::getLeftX,
                controller::getLeftY,
                () -> controller.getRawAxis(4));
        gripper = new Gripper(new GripperIO() {}, new GripperSensorIO() {});
        arm = new Arm(new ArmIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO[] {});
        break;
    }

    m_controllerLeftX = controller::getLeftX;
    m_controllerLeftY = controller::getLeftY;
    m_controllerRightX = () -> controller.getRawAxis(4);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive, m_controllerLeftX, m_controllerLeftY, m_controllerRightX));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    controller.a().onTrue(Commands.runOnce(() -> drive.setDriveState(DriveStates.AUTO_ALIGN)));
    controller.a().onFalse(Commands.runOnce(() -> drive.setDriveState(DriveStates.FIELD_DRIVE)));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed

    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
            : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

    controller.b().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    // Simulation Buttons
    if (Constants.currentMode == Constants.Mode.SIM) {
      GenericHID controllerHID = new GenericHID(0);

      Trigger stupidshit = new Trigger(() -> controllerHID.getRawButton(1));
      Trigger stupidshit2 = new Trigger(() -> controllerHID.getRawButton(2));
      Trigger stupidshit3 = new Trigger(() -> controllerHID.getRawButton(3));

      stupidshit.onTrue(Commands.runOnce(() -> elevator.setState(ElevatorStates.CORAL_L4)));
      stupidshit2.onTrue(
          Commands.runOnce(() -> elevator.setState(ElevatorStates.CORAL_L3_SCORE)));
      stupidshit3.onTrue(
          Commands.runOnce(() -> elevator.setState(ElevatorStates.CORAL_L2_SCORE)));

      // L1 Placement
      controller
          .b()
          .onTrue(
              Commands.runOnce(
                  () ->
                      SimulatedArena.getInstance()
                          .addGamePieceProjectile(
                              new ReefscapeCoralOnFly(
                                  // Obtain robot position from drive simulation
                                  new Translation2d(1, 1),
                                  // The scoring mechanism is installed at (0.46, 0) (meters) on the
                                  // robot
                                  new Translation2d(0, 0),
                                  // Obtain robot speed from drive simulation
                                  new ChassisSpeeds(),
                                  // Obtain robot facing from drive simulation
                                  new Rotation2d(58),
                                  // The height at which the coral is ejected
                                  Distance.ofRelativeUnits(2, Meter),
                                  // The initial speed ofzz the coral
                                  LinearVelocity.ofRelativeUnits(1.5, MetersPerSecond),
                                  // The coral is ejected at a 35-degree slope
                                  Angle.ofRelativeUnits(-45, Degree)))));
    }
    // Human Player

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void periodic() {
    long startTime = RobotController.getFPGATime(); // Get the start time in microseconds
    vision.periodic();
    Logger.recordOutput("diffff", (RobotController.getFPGATime() - startTime) * 1e-3);
    gripper.periodic();
    elevator.periodic();
  }

  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive.setPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }

  public void autonomousInit() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    // Reset the simulation to the initial pose
    driveSimulation.setSimulationWorldPose(drive.getPose());
    gripper.autonomousInit();
  }
}
