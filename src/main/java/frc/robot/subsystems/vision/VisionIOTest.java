package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class VisionIOTest implements VisionIO {
  public VisionIOTest() {
    // This is a test class, so no implementation is needed.
    // In a real implementation, you would initialize the camera and set up the vision system.
  }

  public void updateInputs(VisionIOInputs inputs) {
    // Simulate updating inputs for testing purposes
    inputs.connected = true; // Assume the camera is connected
    inputs.tagIds = new int[] {1};
    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(DriverStation.getMatchTime(), new Pose3d(), 0.1, 1, 1.0, 1.0, 0.01)
        };
    inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
  }

  public String getName() {
    return "VisionIOTest";
  }
}
