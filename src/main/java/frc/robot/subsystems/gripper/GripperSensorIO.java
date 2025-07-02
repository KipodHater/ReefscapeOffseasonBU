package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

public interface GripperSensorIO {

  @AutoLog
  public static class GripperSensorIOInputs {
    public boolean isConnected = false;

    public boolean hasGamepiece = false;
  }

  public default void updateInputs(GripperSensorIOInputs inputs) {}
}