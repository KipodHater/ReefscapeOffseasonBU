package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotState {

  private static RobotState instance;
  private ScoringInfo curScoringInfo = new ScoringInfo(0, false, null, null, false);

  private RobotState(Pose2d initialPose) {}

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState(new Pose2d(new Translation2d(1, 4), new Rotation2d()));
    }
    return instance;
  }

  public void setUpScoringTargetCoral() {
    // getPose
    // find closest reef face and whether should score on backside or frontside
    // save the target reef face and back side or front side
    // update scoring info
    // dont forget alliance flipping
  }

  public void switchScoreSide() {
    // change the poses here
    curScoringInfo =
        new ScoringInfo(
            curScoringInfo.reefFace,
            curScoringInfo.backside,
            curScoringInfo.alignPose,
            curScoringInfo.scorePose,
            !curScoringInfo.shouldScoreRightSide);
  }

  public ScoringInfo getCoralScoringInfo() {
    return curScoringInfo;
  }

  public record ScoringInfo(
      int reefFace,
      boolean backside,
      Pose2d alignPose,
      Pose2d scorePose,
      boolean shouldScoreRightSide) {}
}
