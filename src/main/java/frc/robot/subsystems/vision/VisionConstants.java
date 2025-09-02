package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final short lowResPipeline = 1;
  public static final short highResPipeline = 0;

  public static final double lowResTriggerDistance = 3.0; // meters
  public static final double highResTriggerDistance = 3.5; // meters

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.12, 0.32, 0.5, new Rotation3d(0.0, 20 * Math.PI / 180, 0));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          0.36, -0.36, 0.1, new Rotation3d(0.0, -32 * Math.PI / 180, 15 * Math.PI / 180));

  public static Transform3d[] robotToCameras = {robotToCamera0, robotToCamera1, new Transform3d()};

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.4;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        0.1, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
