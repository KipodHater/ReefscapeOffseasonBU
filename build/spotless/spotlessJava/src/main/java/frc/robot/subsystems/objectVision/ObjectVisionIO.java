package frc.robot.subsystems.objectVision;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectVisionIO {

  public Transform3d robotToCamera = new Transform3d();

  @AutoLog
  public static class ObjectVisionIOInputs {
    public boolean connected = false;
    public TargetObservation[] targets = new TargetObservation[0];
    public double timestamp = 0.0;
  }

  public default void updateInputs(ObjectVisionIOInputs inputs) {}

  public default String getName() {
    return null;
  }

  public static record TargetObservation(
      double tx, double ty, Transform3d robotToCamera, double timestamp) {}
}
