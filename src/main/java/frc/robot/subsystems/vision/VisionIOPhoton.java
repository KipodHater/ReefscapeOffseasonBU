package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

public class VisionIOPhoton implements VisionIO {

  protected final PhotonCamera camera;
  private final Transform3d robotToCamera;
  private Transform3d cameraToTarget, fieldToCamera, fieldToRobot, fieldToTarget;
  private final String name;
  protected boolean isHighRes = true;

  // @AutoLogOutput (key = "Vision/" + name + "/Mode")
  // private VisionIO.hardwareMode mode = VisionIO.hardwareMode.SEARCHING_FOR_TAGS;

  public VisionIOPhoton(String name, Transform3d robotToCamera) {
    this.name = name;
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    inputs.connected = camera.isConnected();

    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    var results = camera.getAllUnreadResults();

    for (var result : results) {
      if (result.hasTargets()
          && RobotController.getFPGATime() * 1e-6 - result.getTimestampSeconds() < 0.02) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      if (result.multitagResult.isPresent()) {
        var multitagResult = result.multitagResult.get();

        fieldToCamera = multitagResult.estimatedPose.best;
        fieldToRobot = robotToCamera.inverse().plus(fieldToCamera); // vector addition
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        double totalTagDistance = 0.0;
        double minTagDistance = 9999.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
          minTagDistance =
              Math.min(minTagDistance, target.bestCameraToTarget.getTranslation().getNorm());
        }

        tagIds.addAll(multitagResult.fiducialIDsUsed);

        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(),
                robotPose,
                multitagResult.estimatedPose.ambiguity,
                multitagResult.fiducialIDsUsed.size(),
                totalTagDistance / multitagResult.fiducialIDsUsed.size(),
                minTagDistance,
                0.1) // add here a function that can calculate the FOM
            );

      } else if (!result.targets.isEmpty()) { // Single tag result
        var target = result.targets.get(0);

        // Calculate robot pose
        var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          cameraToTarget = target.bestCameraToTarget;
          fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          fieldToRobot = fieldToCamera.plus(robotToCamera.inverse()); // vector addition
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          // Add tag ID
          tagIds.add((short) target.fiducialId);

          // Add observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  robotPose, // 3D pose estimate
                  target.poseAmbiguity, // Ambiguity
                  1, // Tag count
                  cameraToTarget.getTranslation().getNorm(), // Average tag distance
                  cameraToTarget.getTranslation().getNorm(), // Minimum tag distance
                  0.1)); // Observation type
        }
      }
    }

    if (!poseObservations.isEmpty()) {
      inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
      if (inputs.poseObservations[poseObservations.size() - 1].minimumTagDistance()
          < lowResTriggerDistance) {
        setHighResPipeline(false); // Switch to low res if the last observation is close enough
      } else if (inputs.poseObservations[poseObservations.size() - 1].minimumTagDistance()
          > highResTriggerDistance) {
        setHighResPipeline(true); // Switch to high res if the last observation is far enough
      }
    } else {
      inputs.poseObservations = new PoseObservation[0];
      tagIds.clear();
    }

    inputs.tagIds = new int[tagIds.size()];

    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
    // Log the time in seconds

  }

  protected void setHighResPipeline(boolean highRes) {
    if (highRes == isHighRes) return; // No change needed
    isHighRes = highRes;
    camera.setPipelineIndex(highRes ? highResPipeline : lowResPipeline);
  }

  public String getName() {
    return name;
  }
}
