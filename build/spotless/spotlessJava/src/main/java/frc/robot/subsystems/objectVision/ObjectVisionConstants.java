package frc.robot.subsystems.objectVision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class ObjectVisionConstants {
  public static Transform3d ROBOT_TO_CAMERA =
      new Transform3d(0.25, 0, 0.5, new Rotation3d(0, -0.5, 0)); // TODO: change to actual value
  public static double CORAL_HEIGHT = 0.5; // TODO: change to actual value
}
