package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotState.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.AllianceFlipping;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {

  private final double txTyObservationStaleSecs = 0.2; // seconds
  private final double minDistanceTagPoseBlend = 0.4; // meters
  private final double maxDistanceTagPoseBlend = 0.9; // meters

  private static RobotState instance;
  private ScoringInfo curCoralScoringInfo = new ScoringInfo(0, false, null, null);
  private ScoringInfo curAlgaeScoringInfo = new ScoringInfo(0, false, null, null);

  private double elevatorOverallHeight = 0.0; // meters

  public static RobotState getInstance() {
    // System.out.println(1);
    if (instance == null) {
      // System.out.println(2);
      instance = new RobotState();
    }
    // System.out.println(3);
    return instance;
  }

  private static final Map<Integer, Pose2d> tagPoses2d = new HashMap<>();

  static {
    for (int i = 1; i <= FieldConstants.aprilTagCount; i++) {
      tagPoses2d.put(
          i,
          FieldConstants.aprilTagLayout.getTagPose(i).map(Pose3d::toPose2d).orElse(Pose2d.kZero));
    }
  }

  @AutoLogOutput private Pose2d odometryPose = Pose2d.kZero;
  @AutoLogOutput private Pose2d estimatedPose = Pose2d.kZero;

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE);

  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

  private final SwerveDriveKinematics kinematics;
  private SwerveModulePosition[] lastWheelPosition =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private Rotation2d gyroOffset = new Rotation2d();

  private final Map<Integer, TxTyPoseRecord> txTyPoses = new HashMap<>();
  private Set<coralPoseRecord> coralPoses = new HashSet<>();

  @AutoLogOutput(key = "RobotState/RobotVelocity")
  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  // elevator extension

  // intake extension

  // arm position

  private RobotState() {
    for (int i = 0; i < 3; i++) {
      qStdDevs.set(i, 0, Math.pow(i, i)); // change this!
    }

    kinematics = new SwerveDriveKinematics(Drive.getModuleTranslations());

    // maybe add here vision standart devs idk wtf
  }

  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  public ChassisSpeeds getRobotVelocity() {
    return robotVelocity;
  }

  public void resetPose(Pose2d newPose) {
    gyroOffset = newPose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
    estimatedPose = newPose;
    odometryPose = newPose;
    poseBuffer.clear();
  }

  public Pose2d getEstimatedPose(double timestamp) {
    var sample = poseBuffer.getSample(timestamp);
    // If pose buffer is empty, return the current estimated pose
    if (sample.isEmpty()) {
      return estimatedPose;
    }
    return sample.get();
  }

  public void addOdometryObservation(OdometryObservation observation) {
    Twist2d twist = kinematics.toTwist2d(lastWheelPosition, observation.wheelPositions());
    lastWheelPosition = observation.wheelPositions();
    Pose2d lastOdometryPose = odometryPose;
    odometryPose = odometryPose.exp(twist);

    observation.gyroAngle.ifPresent(
        angle -> {
          Rotation2d gyroRotation = angle.minus(gyroOffset);
          odometryPose = new Pose2d(odometryPose.getTranslation(), gyroRotation);
        });
    // add to pose buffer
    poseBuffer.addSample(observation.timestamp(), odometryPose);
    Twist2d finalTwist = lastOdometryPose.log(odometryPose);
    estimatedPose = estimatedPose.exp(finalTwist);
  }

  public void addVisionObservation(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (RobotController.getFPGATime() * 1e-6 - POSE_BUFFER_SIZE > timestampSeconds) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }

    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(timestampSeconds);
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }

    // sample --> odometryPose transform and backwards of that
    var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
    var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
    // get old estimate by applying odometryToSample Transform
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    // Calculate 3 x 3 vision matrix
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
    }
    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }
    // difference between estimate and vision pose
    Transform2d transform = new Transform2d(estimateAtTime, visionRobotPoseMeters);
    // scale transform by visionK
    var kTimesTransform =
        visionK.times(
            VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform =
        new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Recalculate current estimate by applying scaled transform to old estimate
    // then replaying odometry data
    estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
  }

  public void addTxTyObservation(
      TxTyObservation observation) { // TODO: change this to our calculation if doesn't work well
    // Skip if current data for tag is newer
    if (txTyPoses.containsKey(observation.tagId())
        && txTyPoses.get(observation.tagId()).timestamp() >= observation.timestamp()) {
      return;
    }

    // Get rotation at timestamp
    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }
    Rotation2d robotRotation =
        estimatedPose.transformBy(new Transform2d(odometryPose, sample.get())).getRotation();

    // Average tx's and ty's
    double tx = 0.0;
    double ty = 0.0;
    for (int i = 0; i < 4; i++) {
      tx += observation.tx()[i];
      ty += observation.ty()[i];
    }
    tx /= 4.0;
    ty /= 4.0;

    Pose3d cameraPose = new Pose3d().plus(VisionConstants.robotToCameras[observation.camera()]);

    // Use 3D distance and tag angles to find robot pose
    Translation2d camToTagTranslation =
        new Pose3d(Translation3d.kZero, new Rotation3d(0, ty, -tx))
            .transformBy(
                new Transform3d(new Translation3d(observation.distance(), 0, 0), Rotation3d.kZero))
            .getTranslation()
            .rotateBy(new Rotation3d(0, cameraPose.getRotation().getY(), 0))
            .toTranslation2d();

    Rotation2d camToTagRotation =
        robotRotation.plus(
            cameraPose.toPose2d().getRotation().plus(camToTagTranslation.getAngle()));
    var tagPose2d = tagPoses2d.get(observation.tagId());
    if (tagPose2d == null) return;
    Translation2d fieldToCameraTranslation =
        new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
            .transformBy(new Transform2d(camToTagTranslation.getNorm(), 0.0, new Rotation2d()))
            .getTranslation();
    Pose2d robotPose =
        new Pose2d(
                fieldToCameraTranslation, robotRotation.plus(cameraPose.toPose2d().getRotation()))
            .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));
    // Use gyro angle at time for robot rotation
    robotPose = new Pose2d(robotPose.getTranslation(), robotRotation);

    // Add transform to current odometry based pose for latency correction
    txTyPoses.put(
        observation.tagId(),
        new TxTyPoseRecord(robotPose, camToTagTranslation.getNorm(), observation.timestamp()));
  }

  public void addDriveSpeeds(ChassisSpeeds speeds) {
    robotVelocity = speeds;
  }

  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
  }

  public ReefPoseEstimate getReefPose(int face, Pose2d finalPose) {
    final boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    var tagPose =
        getTxTyPose(
            switch (face) {
              case 1 -> isRed ? 6 : 19;
              case 2 -> isRed ? 11 : 20;
              case 3 -> isRed ? 10 : 21;
              case 4 -> isRed ? 9 : 22;
              case 5 -> isRed ? 8 : 17;
                // 0
              default -> isRed ? 7 : 18;
            });
    // Use estimated pose if tag pose is not present
    if (tagPose.isEmpty()) return new ReefPoseEstimate(getEstimatedPose(), 0.0);
    // Use distance from estimated pose to final pose to get t value
    final double t =
        MathUtil.clamp(
            (getEstimatedPose().getTranslation().getDistance(finalPose.getTranslation())
                    - minDistanceTagPoseBlend)
                / (maxDistanceTagPoseBlend - minDistanceTagPoseBlend),
            0.0,
            1.0);
    return new ReefPoseEstimate(getEstimatedPose().interpolate(tagPose.get(), 1.0 - t), 1.0 - t);
  }

  /** Get 2d pose estimate of robot if not stale. */
  public Optional<Pose2d> getTxTyPose(int tagId) {
    if (!txTyPoses.containsKey(tagId)) {
      return Optional.empty();
    }
    var data = txTyPoses.get(tagId);
    // Check if stale
    if (Timer.getTimestamp() - data.timestamp() >= txTyObservationStaleSecs) {
      return Optional.empty();
    }
    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(data.timestamp());
    // Latency compensate
    return sample.map(pose2d -> data.pose().plus(new Transform2d(pose2d, odometryPose)));
  }

  public void addCoralPose(coralPoseRecord coralPose) {

    coralPoses =
        coralPoses.stream()
            .filter(
                (x) ->
                    x.translation.getDistance(coralPose.translation())
                        > 0.2) // when to merge corals to one
            .collect(Collectors.toSet());
    // Add to set
    coralPoses.add(new coralPoseRecord(coralPose.translation(), coralPose.timestamp()));
  }

  public Optional<Translation2d> getBestCoral() {

    double minScore = Double.POSITIVE_INFINITY;
    Translation2d bestCoral = Translation2d.kZero;
    boolean noCoralFlag = true;

    for (var coralPose : coralPoses) {
      Translation2d difference = coralPose.translation.minus(estimatedPose.getTranslation());
      double score =
          Math.abs(difference.getAngle().getDegrees() - estimatedPose.getRotation().getDegrees())
                  * 1
              + difference.getNorm()
                  * 40; // can change the weights if selection method doesnt work well
      if (score < minScore) {
        minScore = score;
        noCoralFlag = false;
      }
    }
    if (!noCoralFlag) {
      return Optional.of(bestCoral);
    } else {
      return Optional.empty();
    }
  }

  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  public void periodic() {
    coralPoses.removeIf(
        (coralPose) ->
            Timer.getFPGATimestamp() - coralPose.timestamp()
                > 1); // can change this to something different

    // Log coral posses
    Logger.recordOutput(
        "RobotState/CoralPoses",
        coralPoses.stream()
            .map(
                coralPose ->
                    new Pose3d(
                        new Translation3d(
                            coralPose.translation().getX(),
                            coralPose.translation().getY(),
                            FieldConstants.coralDiameter / 2.0),
                        new Rotation3d()))
            .toArray(Pose3d[]::new));
  }

  public int getClosestReefFaceCoral() {
    // closest is 0 right and then clockwise positive
    double minDistance = Double.POSITIVE_INFINITY;
    int closestFace = 0;
    for (int i = 0; i < 12; i++) {
      double distance =
          AllianceFlipping.apply(CORAL_SCORE_POSES[i].getTranslation())
              .getDistance(estimatedPose.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestFace = i;
      }
    }
    return closestFace;
  }

  public void setUpScoringTargetCoral() {
    int closestFace = getClosestReefFaceCoral();
    boolean isBackSide = true;

    Pose2d alignPose = AllianceFlipping.apply(CORAL_ALIGN_POSES[closestFace]);
    Pose2d scorePose = AllianceFlipping.apply(CORAL_SCORE_POSES[closestFace]);
    double angleError = estimatedPose.getRotation().minus(scorePose.getRotation()).getDegrees();
    angleError = MathUtil.inputModulus(angleError, -180, 180);
    if (angleError > SWITCH_SCORE_FRONT_THRESHOLD) {
      alignPose =
          new Pose2d(
              alignPose.getTranslation(), alignPose.getRotation().plus(new Rotation2d(Math.PI)));
      scorePose =
          new Pose2d(
              scorePose.getTranslation(), scorePose.getRotation().plus(new Rotation2d(Math.PI)));
      isBackSide = false;
    }

    curCoralScoringInfo =
        new ScoringInfo(closestFace, isBackSide, alignPose, scorePose); // even faces are right side
  }

  public void switchScoreSideCoral() {
    int newReefFace =
        curCoralScoringInfo.reefFace() + curCoralScoringInfo.reefFace() % 2 == 0 ? 1 : -1;

    Pose2d alignPose = AllianceFlipping.apply(CORAL_ALIGN_POSES[newReefFace]);
    Pose2d scorePose = AllianceFlipping.apply(CORAL_SCORE_POSES[newReefFace]);

    if (!curCoralScoringInfo.backside()) {
      alignPose.rotateBy(Rotation2d.fromDegrees(180));
      scorePose.rotateBy(Rotation2d.fromDegrees(180));
    }

    curCoralScoringInfo =
        new ScoringInfo(newReefFace, curCoralScoringInfo.backside, alignPose, scorePose);
  }

  public ScoringInfo getCoralScoringInfo() {
    return curCoralScoringInfo;
  }

  public int getClosestReefFaceAlgae() {
    // closest is 0 and then clockwise positive
    double minDistance = Double.POSITIVE_INFINITY;
    int closestFace = 0;
    for (int i = 0; i < 6; i++) {
      double distance =
          AllianceFlipping.apply(ALGAE_INTAKE_POSES[i].getTranslation())
              .getDistance(estimatedPose.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestFace = i;
      }
    }
    return closestFace;
  }

  public void setUpReefAlgae() {
    int closestFace = getClosestReefFaceAlgae();
    System.out.println(closestFace);
    boolean isBackSide = true;

    Pose2d alignPose = AllianceFlipping.apply(ALGAE_ALIGN_POSES[closestFace]);
    Pose2d scorePose = AllianceFlipping.apply(ALGAE_INTAKE_POSES[closestFace]);
    double angleError = estimatedPose.getRotation().minus(scorePose.getRotation()).getDegrees();
    angleError = MathUtil.inputModulus(angleError, -180, 180);
    if (angleError > SWITCH_SCORE_FRONT_THRESHOLD) {
      alignPose.rotateBy(Rotation2d.fromDegrees(180));
      scorePose.rotateBy(Rotation2d.fromDegrees(180));
      isBackSide = false;
    }

    curAlgaeScoringInfo = new ScoringInfo(closestFace, isBackSide, alignPose, scorePose);
  }

  public ScoringInfo getAlgaeScoringInfo() {
    return curAlgaeScoringInfo;
  }

  public void setElevatorOverallHeight(double elevatorOverallHeight) {
    this.elevatorOverallHeight = elevatorOverallHeight;
  }

  public double getElevatorOverallHeight() {
    return elevatorOverallHeight;
  }

  public record TxTyObservation(
      int tagId, int camera, double[] tx, double[] ty, double distance, double timestamp) {}

  public record OdometryObservation(
      SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}

  public record coralPoseRecord(Translation2d translation, double timestamp) {}

  public record TxTyPoseRecord(Pose2d pose, double distance, double timestamp) {}

  public record ScoringInfo(int reefFace, boolean backside, Pose2d alignPose, Pose2d scorePose) {}

  public record ReefPoseEstimate(Pose2d pose, double blend) {}
}
