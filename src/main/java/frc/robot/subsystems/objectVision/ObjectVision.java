package frc.robot.subsystems.objectVision;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.objectVision.ObjectVisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.coralPoseRecord;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class ObjectVision extends SubsystemBase {

  // private final ObjectVisionConsumer consumer;

  private final ObjectVisionIO io;
  private final ObjectVisionIOInputsAutoLogged inputs;
  private final Alert alerts;

  public ObjectVision(ObjectVisionIO io) {

    this.io = io;
    this.inputs = new ObjectVisionIOInputsAutoLogged();
    // inputs = new ObjectVisionIOInputsAutoLogged();

    alerts =
        new Alert(
            "Object detection camera: " + io.getName() + " is disconnected.",
            Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ObjectVision/Camera " + io.getName(), inputs);
    alerts.set(!inputs.connected);

    List<Pose2d> targetTranslations = new LinkedList<>();
    List<Pose2d> targetTranslationsRejected = new LinkedList<>();
    List<Pose2d> targetTranslationsAccepted = new LinkedList<>();
    // Logger.recordOutput("ObjectVision/TargetLength", io.length);
    // for (int i = 0; i < io.length; i++) {
    Pose2d fieldToRobot = RobotState.getInstance().getEstimatedPose(inputs.timestamp);
    // Transform3d robotToCamera = robotToCamera;
    Rotation2d cameraPitch = new Rotation2d(ROBOT_TO_CAMERA.getRotation().getX());
    Rotation2d cameraYaw = new Rotation2d(ROBOT_TO_CAMERA.getRotation().getZ());
    Logger.recordOutput("ObjectVision/TargetLength", inputs.targets.length);

    for (var target : inputs.targets) {

      Rotation2d yaw =
          fieldToRobot
              .getRotation()
              .plus(projectToGround(cameraPitch, Rotation2d.fromDegrees(target.tx())))
              .plus(cameraYaw);
      if (Math.abs(cameraPitch.plus(Rotation2d.fromDegrees(target.ty())).getCos()) < 0.01) continue;
      double distance =
          (ROBOT_TO_CAMERA.getZ() - CORAL_HEIGHT)
              * (cameraPitch.plus(Rotation2d.fromDegrees(target.ty()))).getTan();

      Translation2d fieldToTarget =
          new Translation2d(
              fieldToRobot.getX() + distance * yaw.getCos(),
              fieldToRobot.getY() + distance * yaw.getSin());

      // Translation2d targetTranslation = fieldToRobot.getTranslation().plus(robotToTarget);

      Pose2d targetPose = new Pose2d(fieldToTarget, new Rotation2d());

      targetTranslations.add(targetPose);
    }

    // Pose2d[] arrayResults = new Pose2d[targetTranslations.size()];
    for (var targetPose : targetTranslations) {
      // Check whether to reject pose
      boolean rejectPose =
          targetPose.getX() < 0.0
              || targetPose.getX() > FieldConstants.fieldLength
              || targetPose.getY() < 0.0
              || targetPose.getY() > FieldConstants.fieldWidth;

      if (rejectPose) {
        targetTranslationsRejected.add(targetPose);
      } else {
        targetTranslationsAccepted.add(targetPose);
        RobotState.getInstance()
            .addCoralPose(new coralPoseRecord(targetPose.getTranslation(), inputs.timestamp));
      }
    }

    Logger.recordOutput(
        "ObjectVision/AllTargetPoses",
        targetTranslations.toArray(new Translation2d[targetTranslations.size()]));
    Logger.recordOutput(
        "ObjectVision/AllTargetPoses",
        targetTranslationsRejected.toArray(new Translation2d[targetTranslationsRejected.size()]));
    Logger.recordOutput(
        "ObjectVision/AllTargetPoses",
        targetTranslationsAccepted.toArray(new Translation2d[targetTranslationsAccepted.size()]));
  }

  private Rotation2d projectToGround(Rotation2d cameraPitch, Rotation2d tx) {
    // if (Math.abs(tx.getDegrees()) < (1e-6)) return new Rotation2d();
    return new Rotation2d(Math.atan2(tx.getTan(), cameraPitch.getCos()));
  }

  // @FunctionalInterface
  // public static interface ObjectVisionConsumer {
  //     public void accept(
  //         Transform2d robotToTarget,
  //         double timestampSeconds);
  // }
}
