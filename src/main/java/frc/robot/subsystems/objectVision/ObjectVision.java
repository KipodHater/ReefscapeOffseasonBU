package frc.robot.subsystems.objectVision;

import static frc.robot.subsystems.objectVision.ObjectVisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ObjectVision extends SubsystemBase {

  // private final ObjectVisionConsumer consumer;

  private final ObjectVisionIO io;
  private final ObjectVisionIOInputsAutoLogged inputs;
  private final Supplier<Pose2d> poseFunction;
  private final Alert alerts;

  public ObjectVision(Supplier<Pose2d> poseFunction, ObjectVisionIO io) {

    this.io = io;
    this.inputs = new ObjectVisionIOInputsAutoLogged();
    // inputs = new ObjectVisionIOInputsAutoLogged();

    this.poseFunction = poseFunction;

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
    // Logger.recordOutput("ObjectVision/TargetLength", io.length);
    // for (int i = 0; i < io.length; i++) {
    Pose2d fieldToRobot = poseFunction.get(); // .apply(inputs[i].timestamp);
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
      double distance =
          (ROBOT_TO_CAMERA.getZ() - CORAL_HEIGHT)
              * (cameraPitch.plus(Rotation2d.fromDegrees(target.ty()))).getTan();

      Logger.recordOutput("ObjectVision/Distance", distance);
      Translation2d fieldToTarget =
          new Translation2d(
              fieldToRobot.getX() + distance * yaw.getCos(),
              fieldToRobot.getY() + distance * yaw.getSin());

      // Translation2d targetTranslation = fieldToRobot.getTranslation().plus(robotToTarget);

      Pose2d targetPose = new Pose2d(fieldToTarget, new Rotation2d());
      // TODO: add here clamping target values into field
      Logger.recordOutput("ObjectVision/TargetLength", 1);
      targetTranslations.add(targetPose);
    }

    Pose2d[] arrayResults = new Pose2d[targetTranslations.size()];
    int i = 0;
    for (var targetPose : targetTranslations) {
      arrayResults[i++] = targetPose;
    }
    // Logger.recordOutput("ObjectVision/TargetLength", arrayResults.length);
    Logger.recordOutput("ObjectVision/TargetPoses", arrayResults);
  }

  private Rotation2d projectToGround(Rotation2d cameraPitch, Rotation2d tx) {
    if (Math.abs(tx.getDegrees()) < (1e-6)) return new Rotation2d();
    return new Rotation2d(Math.atan(1.0 * cameraPitch.getTan() / tx.getCos()));
  }

  // @FunctionalInterface
  // public static interface ObjectVisionConsumer {
  //     public void accept(
  //         Transform2d robotToTarget,
  //         double timestampSeconds);
  // }
}
