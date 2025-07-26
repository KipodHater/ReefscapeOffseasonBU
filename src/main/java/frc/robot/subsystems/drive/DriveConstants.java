package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {

  public static final double robotMassKg = 43.0; // Mass of the robot in kg
  public static final double robotMOI = 6.883; // TODO: calculate MOI
  public static final double wheelCOF = 1.0;

  public static final double maxSpeedMetersPerSec = 5.2;
  public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = 0.55;
  public static final double wheelBase = 0.55;
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
      };

  public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);

  public static final double maxSpeedMPS = 5.3;

  public static final DCMotor driveGearbox = DCMotor.getFalcon500(1);
  public static final DCMotor turnGearbox = DCMotor.getFalcon500(1);
  public static final double driveMotorReduction = 5.9; // Gear ratio for drive motor
  public static final double turnMotorReduction = 18.75; // Gear ratio for turn motor

  public static final double DEADBAND = 0.1; // Deadband for joystick inputs

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withCustomModuleTranslations(moduleTranslations)
          .withRobotMass(Mass.ofRelativeUnits(robotMassKg, Kilogram))
          .withGyro(COTS.ofNav2X())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  driveGearbox,
                  turnGearbox,
                  driveMotorReduction,
                  turnMotorReduction,
                  Volts.of(0.1),
                  Volts.of(0.1),
                  Meters.of(wheelRadiusMeters),
                  KilogramSquareMeters.of(0.02),
                  wheelCOF));
}
