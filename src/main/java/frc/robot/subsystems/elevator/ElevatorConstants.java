package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;

public class ElevatorConstants { // TODO: tune everything
    public static final double ELEVATOR_MAX_VELOCITY = 20, ELEVATOR_MAX_ACCELERATION = 40; // deg/s, deg/s2
  public static final Constraints ELEVATOR_CONSTRAINTS =
      new Constraints(ELEVATOR_MAX_VELOCITY, ELEVATOR_MAX_ACCELERATION);

  public static final Gains GAINS =
      switch (Constants.currentMode) {
        case REAL -> new Gains(0, 0, 0, 0, 0, 0.0, 0);
        default -> new Gains(0.1, 0, 0, 0, 0, 0, 0);
      };

  public static final double ELEVATOR_GEAR_RATIO = 0; 
  public static final double ELEVATOR_LENGTH_METERS = 0; // meters, change!
  public static final double ELEVATOR_MOI = 0; // kg*m^2

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
