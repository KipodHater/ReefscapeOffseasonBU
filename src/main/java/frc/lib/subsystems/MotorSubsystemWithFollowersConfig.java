package frc.lib.subsystems;

public class MotorSubsystemWithFollowersConfig extends MotorSubsystemConfig {
  public static class FollowerConfig {
    public MotorSubsystemConfig config = new MotorSubsystemConfig();
    public boolean inverted = false;
  }

  public FollowerConfig[] followerConfigs = new FollowerConfig[0];
}
