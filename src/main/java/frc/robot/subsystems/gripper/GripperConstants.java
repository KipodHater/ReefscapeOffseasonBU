package frc.robot.subsystems.gripper;

import frc.robot.Constants;

public class GripperConstants {

  public static final Gains GAINS =
      switch (Constants.currentMode) {
        case REAL -> new Gains(0.1, 0, 0, 0, 0, 0);
        default -> new Gains(0.1, 0, 0, 0, 0, 0);
      };

  public static final double KMAX_ACCEL = 0.5;
  public static final double KMAX_SPEED = 1;

  public static final double GRIPPER_OUTTAKE_SPEED = -0.29;
  public static final double GRIPPER_OUTTAKEFAST_SPEED = -0.37;
  public static final double GRIPPER_INTAKE_SPEED = 0.85;

  public static final int K_SPARK_ID = 18;
  // public static final int K_BEAMBREAK_ID = 1;
  public static final int K_CURRENT_LIMIT = 65; // amps

  public static final boolean K_INVERTED = true;
  public static final boolean K_BRAKE = true;

  public static final double VELOCITY_CONVERSION_FACTOR = 1;

  public static final double GEAR_RATIO = 1;

  public static final double GRIPPER_MOMENT_OF_INNERTIA = 0.6; // kg* m^2

  public record Gains(double KP, double KI, double KD, double KS, double KV, double KA) {}
}
