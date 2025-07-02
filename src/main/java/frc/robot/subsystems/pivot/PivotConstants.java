package frc.robot.subsystems.pivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;

public class PivotConstants {
  public static final double MAX_VELOCITY = 150, MAX_ACCELARATION = 550; // deg/s, deg/s2
  public static final Constraints PIVOT_CONSTRAINTS =
      new Constraints(MAX_VELOCITY, MAX_ACCELARATION);

  public static final Gains GAINS;

  static {
    switch (Constants.currentMode) {
      case REAL:
        GAINS = new Gains(0.065, 0, 0, 0.01, 0.01, 0.0, 0.04);
        break;
      default:
        GAINS = new Gains(0.2, 0, 0, 0, 0, 0, 0);
        break;
    }
  }

  public static final double GEAR_RATIO = 1; // TODO: change
  public static final double PIVOT_LENGTH = 0.2; // meters, change!
  public static final double PIVOT_MOI = 0.4; // kg*m^2
  public static final double MAX_ANGLE = 92.0; // degrees
  public static final double MIN_ANGLE = -30.0; // degrees

  public static final int CURRENT_LIMIT = 50;
  public static final double ENCODER_OFFSET = 207.2 - 1.5 - 4.8 + 118;
  public static final double POSITION_TOLERANCE = 1.0;
  public static final boolean PIVOT_BRAKE = true;
  public static final double POSITION_CONVERSION_FACTOR = 360; // makes it degrees
  public static final double VELOCITY_CONVERSION_FACTOR = 60; // makes it rpm

  public static final int MOTOR_ID = 19;

  public static final boolean MOTOR_INVERTED = false;

  public static final double MID_ANGLE = 92.0;
  public static final double TOP_ANGLE = 152.5;
  public static final double BOT_ANGLE = 117.5;
  public static final double CLIMB_ANGLE = 117.5;

  public record Gains(
      double KP, double KI, double KD, double KS, double KV, double KA, double KG) {}
}
