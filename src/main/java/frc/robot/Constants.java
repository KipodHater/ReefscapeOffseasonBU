// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double CYCLE_TIME = 0.02;
  public static final double FIELD_LENGTH = 16.5;
  public static final double FIELD_WIDTH = 8.3;
  public static final double POSE_BUFFER_SIZE = 2.0; // seconds

  public static class FieldConstants {

    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final double fieldLength = aprilTagLayout.getFieldLength();

    public static final double fieldWidth = aprilTagLayout.getFieldLength();
    ;
    public static final double startingLineX =
        Units.inchesToMeters(299.438); // Measured from the inside of starting line
    public static final double algaeDiameter = Units.inchesToMeters(16);
    public static final double coralDiameter = Units.inchesToMeters(4.5);
    public static final int aprilTagCount = 22;

    public static class Processor {
      public static final Pose2d centerFace =
          new Pose2d(aprilTagLayout.getTagPose(16).get().getX(), 0, Rotation2d.fromDegrees(90));
      public static final Pose2d opposingCenterFace =
          new Pose2d(
              aprilTagLayout.getTagPose(3).get().getX(), fieldWidth, Rotation2d.fromDegrees(-90));
    }
  }

  public final class RobotState {

    public static final double SWITCH_SCORE_FRONT_THRESHOLD = 100; // deg

    public static final Pose2d[] CORAL_SCORE_POSES = {
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(5.3, 2.5, Rotation2d.fromDegrees(-60))
    };

    public static final Pose2d[] CORAL_ALIGN_POSES = {
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(5.1, 2.7, Rotation2d.fromDegrees(-60))
    };

    public static final Pose2d[] ALGAE_ALIGN_POSES = {
      new Pose2d(3, 3, new Rotation2d()),
      new Pose2d(3, 3, new Rotation2d()),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d()
    };

    public static final Pose2d[] ALGAE_INTAKE_POSES = {
      new Pose2d(3, 3, new Rotation2d()),
      new Pose2d(3, 3, new Rotation2d()),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d()
    };
  }
}
