package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class RobotState {

  private static RobotState instance;

  private Pose2d currentPose;
  private Pose2d previousPose;

  private Translation2d robotTranslation;
  private Rotation2d robotYaw;

  private double translationFOM; // m
  private double angleFOM; // deg

  private RobotState(Pose2d initialPose) {
    // Prevent instantiation
    currentPose = initialPose;
    previousPose = initialPose;

    translationFOM = 0.0;
    angleFOM = 0.0;
  }

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState(new Pose2d(new Translation2d(1, 4), new Rotation2d()));
    }
    return instance;
  }

  public void resetPose(Pose2d pose) {
    currentPose = pose;
    previousPose = pose;

    translationFOM = 0.05;
    angleFOM = 1;
  }

  public void resetRotation(Rotation2d newRotation) {
    currentPose = new Pose2d(currentPose.getTranslation(), newRotation);
    previousPose = new Pose2d(currentPose.getTranslation(), newRotation);

    angleFOM = 0.;
  }

  public void resetGyro(Rotation2d newRotation) {
    currentPose = new Pose2d(currentPose.getTranslation(), newRotation);
    previousPose = new Pose2d(currentPose.getTranslation(), newRotation);

    angleFOM = 1.5;
  }

  public void updateOddometryWithTime(
      double Timestamp, Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {}

  public void addVisionMeasurement(Pose2d visionPose2d, double timestampSeconds, double stdDevs) {}

  public Pose2d getEstimatedPosition() {
    return currentPose;
  }

  public Rotation2d getYaw() {
    return currentPose.getRotation();
  }

  public double getVelocity() {
    return 0.0; // insert future implementation
  }
}
