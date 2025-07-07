package frc.robot.subsystems.objectVision;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;

public class ObjectVisionIOPhoton implements ObjectVisionIO {

  private final String name;
  private final PhotonCamera camera;
  public final Transform3d robotToCamera;

  public ObjectVisionIOPhoton(String name, Transform3d robotToCamera) {

    this.name = name;
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  public void updateInputs(ObjectVisionIO.ObjectVisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    inputs.timestamp = camera.getLatestResult().getTimestampSeconds();
    var results = camera.getLatestResult();
    if (results.hasTargets()) {
      inputs.targets = new ObjectVisionIO.TargetObservation[results.targets.size()];
      for (int i = 0; i < results.targets.size(); i++) {

        inputs.targets[i] =
            new ObjectVisionIO.TargetObservation(
                results.targets.get(i).getYaw(),
                results.targets.get(i).getPitch(),
                robotToCamera,
                results.getTimestampSeconds());
      }
    } else {
      inputs.targets = new ObjectVisionIO.TargetObservation[0];
    }
  }

  public String getName() {
    return name;
  }
}
