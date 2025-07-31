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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  public enum DriveStates {
    FIELD_DRIVE,
    ROBOT_DRIVE,
    ASSISTED_DRIVE,
    AUTO_ALIGN,
    AUTONOMOUS,
    SLOWLY_FORWARD,
    IDLE
  }

  @AutoLogOutput(key = "Drive/DriveState")
  private DriveStates driveState = DriveStates.FIELD_DRIVE;

  // TunerConstants doesn't include these constants, so they are declared locally
  // static final double ODOMETRY_FREQUENCY =
  //     new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 45;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getFalcon500(1).withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  // static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  // private SwerveDrivePoseEstimator poseEstimator =
  //     new SwerveDrivePoseEstimator(
  //         kinematics,
  //         rawGyroRotation,
  //         lastModulePositions,
  //         new Pose2d()); // can switch this with potential kalman filter

  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  private final DoubleSupplier xJoystickVelocity, yJoystickVelocity, rJoystickVelocity;

  private Supplier<Pose2d> autoAlignTarget;
  private boolean isBackside = false; // Used for auto-aligning to the reef

  private final ProfiledPIDController linearVelocityController =
      new ProfiledPIDController(5, 0, 0, new Constraints(getMaxLinearSpeedMetersPerSec(), 5)); // kp 1.2
  private final ProfiledPIDController rotationController =
      new ProfiledPIDController(0.8, 0, 0, new Constraints(getMaxAngularSpeedRadPerSec(), 10)); // kp 0.8

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallBack,
      DoubleSupplier xJoystickVelocity,
      DoubleSupplier yJoystickVelocity,
      DoubleSupplier rJoystickVelocity) {

    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);
    this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    // PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        RobotState.getInstance()::resetPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    this.xJoystickVelocity = xJoystickVelocity;
    this.yJoystickVelocity = yJoystickVelocity;
    this.rJoystickVelocity = rJoystickVelocity;
  }

  @Override
  public void periodic() {
    // odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    // odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // // Update odometry
    // double[] sampleTimestamps =
    //     modules[0].getOdometryTimestamps(); // All signals are sampled together
    // int sampleCount = sampleTimestamps.length;
    // for (int i = 0; i < sampleCount; i++) {
    //   // Read wheel positions and deltas from each module
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) { // can add here skid detection
      modulePositions[moduleIndex] = modules[moduleIndex].getPosition();
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    //   // Apply update
    //   poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    // }

    RobotState.getInstance()
        .addOdometryObservation(
            new OdometryObservation(
                modulePositions, Optional.of(rawGyroRotation), RobotController.getFPGATime()));

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    stateMachine();
  }

  public void teleopInit() {
    driveState = DriveStates.FIELD_DRIVE;
  }

  public void autonomousInit() {
    driveState = DriveStates.AUTONOMOUS;
  }

  private void stateMachine() {

    switch (driveState) {
      case IDLE:
        if (xJoystickVelocity.getAsDouble() != 0
            || yJoystickVelocity.getAsDouble() != 0
            || rJoystickVelocity.getAsDouble() != 0) {
          setDriveState(DriveStates.FIELD_DRIVE);
        } else {
          stop();
        }
        break;

      case FIELD_DRIVE:
        fieldCentricJoystickDrive(
            xJoystickVelocity.getAsDouble(),
            yJoystickVelocity.getAsDouble(),
            rJoystickVelocity.getAsDouble());
        break;

      case ROBOT_DRIVE: // Robot drives autonomously
        robotCentricJoystickDrive(
            xJoystickVelocity.getAsDouble(),
            yJoystickVelocity.getAsDouble(),
            rJoystickVelocity.getAsDouble());
        break;

      case ASSISTED_DRIVE: // Field drive but with assistance from the robot
        // This is a placeholder for assisted drive logic
        break;

      case SLOWLY_FORWARD: // Drive backwards slowly
        robotCentricJoystickDrive(
            0,
            0.05 * getMaxLinearSpeedMetersPerSec() * (isBackside ? -1 : 1),
            0); // TODO: check if forward is x or y
        break;

      case AUTO_ALIGN: // Align the robot with reef
        autoAlignTarget = autoAlignTarget != null ? autoAlignTarget : () -> new Pose2d(3, 3, new Rotation2d());
        Transform2d distance = getPose().minus(autoAlignTarget.get());
        double linearVelocity =
            linearVelocityController.calculate(Math.hypot(distance.getX(), distance.getY()), 0);
        Translation2d linearVelocityTranslation =
            new Translation2d(
                linearVelocity, new Rotation2d(Math.atan2(distance.getY(), distance.getX())));
        runVelocity(
            new ChassisSpeeds(
                linearVelocityTranslation.getX(),
                linearVelocityTranslation.getY(),
                rotationController.calculate(
                    getPose().getRotation().getDegrees(),
                    autoAlignTarget.get().getRotation().getDegrees())));
        break;
      default:
        System.out.println("Drive subsystem is really broken");
        break;
    }
  }

  public void autonomousPeriodic(ChassisSpeeds speeds) {
    // This is called every 20ms during autonomous
    // You should call this from the auto command
    runVelocity(speeds);
  }

  public void setDriveState(DriveStates state) {
    if (driveState == state) return;
    driveState = state;
  }

  private void fieldCentricJoystickDrive(double vx, double vy, double vr) {
    Translation2d linearVelocity = getLinearVelocityFromJoysticks(vx, vy);

    double omega = MathUtil.applyDeadband(vr, DriveConstants.DEADBAND);
    // Square rotation value for more precise control
    omega = Math.copySign(omega * omega, omega);

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
            omega * getMaxAngularSpeedRadPerSec());
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, isFlipped ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation()));
  }

  private void robotCentricJoystickDrive(double vx, double vy, double vr) {
    Translation2d linearVelocity = getLinearVelocityFromJoysticks(vx, vy);

    double omega = MathUtil.applyDeadband(vr, DriveConstants.DEADBAND);
    // Square rotation value for more precise control
    omega = Math.copySign(omega * omega, omega);

    runVelocity(
        new ChassisSpeeds(
            linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
            omega * getMaxAngularSpeedRadPerSec()));
  }

  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return RobotState.getInstance().getEstimatedPose();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  private Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DriveConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  public void setState(DriveStates state) {
    if (state == driveState) return;
    driveState = state;
    Logger.recordOutput("Drive/DriveState", driveState.toString());
  }

  public void setStateAutoAlign(Supplier<Pose2d> targetPose) {
    autoAlignTarget = targetPose;
    if (driveState == DriveStates.AUTO_ALIGN) return;
    driveState = DriveStates.AUTO_ALIGN;
  }

  public void setStateSlowlyForward(boolean isBackside) {
    this.isBackside = isBackside;
    if (driveState == DriveStates.SLOWLY_FORWARD) return;
    driveState = DriveStates.SLOWLY_FORWARD;
  }

  public DriveStates getState() {
    return driveState;
  }
}
