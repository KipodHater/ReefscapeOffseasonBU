package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;

public class ElevatorConstants { // TODO: tune everything
  public static final double ELEVATOR_MAX_VELOCITY = 20,
      ELEVATOR_MAX_ACCELERATION = 40; // deg/s, deg/s2
  public static final Constraints ELEVATOR_CONSTRAINTS =
      new Constraints(ELEVATOR_MAX_VELOCITY, ELEVATOR_MAX_ACCELERATION);

  public static final Gains GAINS =
      switch (Constants.currentMode) {
        case REAL -> new Gains(0, 0, 0, 0, 0, 0.0, 0);
        default -> new Gains(5, 0.2, 0, 0, 0.1, 0, 0.05);
      };

  public static final double ELEVATOR_GEAR_RATIO = 1;
  public static final double ELEVATOR_LENGTH_METERS = 1.7; // meters
  public static final double ELEVATOR_MOI = 0; // kg*m^2

  public static final double PULLEY_SOURCE_RADIUS = 0.1; // meters
  public static final double PULLEY_GEAR_RATIO = 1 / 50.0; // 50:1 gear ratio for the pulley

  public static final double ROTATIONS_PER_METER =
      1 / (2 * Math.PI * PULLEY_SOURCE_RADIUS) * PULLEY_GEAR_RATIO * 8; // rotations per meter

  public static final int ELEVATOR_CURRENT_LIMIT = 50;
  public static final double ELEVATOR_ENCODER_OFFSET = 0;
  public static final double ELEVATOR_POSITION_TOLERANCE_DEG = 1.0;
  public static final boolean ELEVATOR_INVERTED = false;
  public static final boolean ELEVATOR_BRAKE = true;
  public static final double POSITION_CONVERSION_FACTOR = 360; // makes it degrees
  public static final double VELOCITY_CONVERSION_FACTOR = 60; // makes it rpm

  public static final int FOLLOWER_ID = 28;
  public static final int MOTOR_ID = 29;

  public static final double STOW_ANGLE = 0; // deg
  public static final double HOME_ANGLE = 0; // deg
  public static final double L2_ANGLE = 0; // deg
  public static final double L3_ANGLE = 0; // deg
  public static final double L4_ANGLE = 0; // deg

  public record Gains(
      double KP, double KI, double KD, double KS, double KV, double KA, double KG) {}
}
