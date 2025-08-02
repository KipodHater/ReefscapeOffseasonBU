package frc.robot.subsystems.intakeDeploy;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;

public class IntakeDeployConstants {

  public static final Gains GAINS =
      switch (Constants.currentMode) {
        case REAL -> new Gains(0.26, 0, 0.01, 0.01, 0.1, 0.0, 0.1);
        default -> new Gains(0.1, 0, 0, 0, 0, 0, 0);
      };

  public static final Constraints CONSTRAINTS = new Constraints(700, 700);

  public static final int MOTOR_ID = 70;
  public static final boolean INTAKE_INVERTED = true;

  public static final double POSITION_CONVERSION_FACTOR = 360.0; // to degrees
  public static final double VELOCITY_CONVERSION_FACTOR = 360.0 / 60.0; // to degrees per second
  public static final double ENCODER_OFFSET = 0.0; // degrees
  public static final boolean ENCODER_INVERTED = false;

  public record Gains(
      double KP, double KI, double KD, double KS, double KV, double KA, double KG) {}
}
