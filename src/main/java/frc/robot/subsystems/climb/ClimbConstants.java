package frc.robot.subsystems.climb;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;

public class ClimbConstants {
  public static final Gains GAINS =
      switch (Constants.currentMode) {
        case REAL -> new Gains(0, 0, 0, 0, 0, 0, 0);
        default -> new Gains(0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };

  public static final double CLIMB_MAX_VELOCITY = 800,
      CLIMB_MAX_ACCELARATION = 1000; // deg/s, deg/s2
  public static final Constraints CLIMB_CONSTRAINTS =
      new Constraints(CLIMB_MAX_VELOCITY, CLIMB_MAX_ACCELARATION);

  public static final double TOLERANCE = 2; // degrees
  public static final int MOTOR_ID = 1;
  public static final boolean MOTOR_INVERTED = false;
  public static final boolean ENCODER_INVERTED = false;

  public static final double POSITION_CONVERSION_FACTOR = 1.0; // degrees per rotation
  public static final double VELOCITY_CONVERSION_FACTOR = 1.0; // degrees per second per rotation

  public static final double CLIMB_ENCODER_OFFSET = 0.0; // degrees

  public record Gains(
      double KP, double KI, double KD, double KS, double KV, double KA, double KG) {}
}
