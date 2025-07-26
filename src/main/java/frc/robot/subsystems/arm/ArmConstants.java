package frc.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;

public class ArmConstants {
  public static final double ARM_MAX_VELOCITY = 800, ARM_MAX_ACCELARATION = 1000; // deg/s, deg/s2
  public static final Constraints ARM_CONSTRAINTS =
      new Constraints(ARM_MAX_VELOCITY, ARM_MAX_ACCELARATION);

  public static final Gains GAINS =
      switch (Constants.currentMode) {
        case REAL -> new Gains(0.26, 0, 0.01, 0.01, 0.1, 0.0, 0.1);
        default -> new Gains(0.1, 0, 0, 0, 0, 0, 0);
      };

  public static final double ARM_GEAR_RATIO = 14.0 / (36.0 * 60.0); // 100:1
  public static final double ARM_LENGTH_METERS = 1.0; // meters, change!
  public static final double ARM_MOI = 1.5; // kg*m^2
  public static final double MAX_ANGLE = 92.0; // degrees
  public static final double MIN_ANGLE = -30.0; // degrees

  public static final int ARM_CURRENT_LIMIT = 50;
  public static final double ARM_ENCODER_OFFSET = 55.0 - 16 - 40 + 74 + 6;
  public static final double ARM_POSITION_TOLERANCE_DEG = 1.0;
  public static final boolean ARM_INVERTED = false;
  public static final boolean ARM_BRAKE = true;
  public static final double POSITION_CONVERSION_FACTOR = 360; // makes it degrees
  public static final double VELOCITY_CONVERSION_FACTOR = 60; // makes it rpm

  public static final int FOLLOWER_ID = 9;
  public static final int MOTOR_ID = 8;

  public static final boolean MOTOR_INVERTED = false;

  public static final double MID_ANGLE = 44.5; // 47.5
  public static final double TOP_ANGLE = 80.0; // 76
  public static final double BOT_ANGLE = -12.5; // 355.5
  public static final double CLIMB_ANGLE = -15;

  public record Gains(
      double KP, double KI, double KD, double KS, double KV, double KA, double KG) {}
}
