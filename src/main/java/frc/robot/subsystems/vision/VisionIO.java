package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  // record visionIOData(
  //   boolean connected,
  //   TargetObservation latestTargetObservation,
  //   PoseObservation[] poseObservations,
  //   int[] tagIds
  // ){}

  public default void updateInputs(VisionIOInputs inputs) {}

  public default String getName() {
    return null;
  }

  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      double minimumTagDistance,
      double measurementFOM) {}
}
