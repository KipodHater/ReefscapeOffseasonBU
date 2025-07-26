package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonSim extends VisionIOPhoton {
  private static VisionSystemSim visionSim;
  private final PhotonCameraSim cameraSim;

  private final Supplier<Pose2d> poseSupplier;

  public VisionIOPhotonSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    SimCameraProperties cameraProp = new SimCameraProperties();
    // add here camera properties:
    cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(75));
    cameraProp.setAvgLatencyMs(10);
    cameraSim = new PhotonCameraSim(camera, cameraProp, aprilTagLayout);

    visionSim.addCamera(cameraSim, robotToCamera);
  }

  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }

  public String getName() {
    return super.getName();
  }

  // @Override
  // protected void setHighResPipeline(boolean highRes) {
  //     if(highRes == isHighRes) return; // No change needed
  //     isHighRes = highRes;
  //     cameraSim.
  // }
}
