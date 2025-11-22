package frc.lib.subsystems;

import org.littletonrobotics.junction.Logger;

public class MotorSubsystemWithFollowers<T extends MotorInputsAutoLogged, U extends MotorIO>
    extends MotorSubsystem<T, U> {
  protected MotorSubsystemWithFollowersConfig leaderConfig;
  protected MotorSubsystemWithFollowersConfig.FollowerConfig[] followerConfigs;

  protected T[] followerInputs;
  protected U[] followerIOs;

  public MotorSubsystemWithFollowers(
      MotorSubsystemWithFollowersConfig leaderConfig,
      T leaderInputs,
      U leaderIO,
      T[] followerInputs,
      U[] followerIOs) {
    super(leaderInputs, leaderIO, leaderConfig);
    this.leaderConfig = leaderConfig;
    this.followerConfigs = leaderConfig.followerConfigs;
    this.followerInputs = followerInputs;
    this.followerIOs = followerIOs;

    this.leaderConfig = leaderConfig;
    this.followerConfigs = leaderConfig.followerConfigs;
    this.followerInputs = followerInputs;
    this.followerIOs = followerIOs;
  }

  @Override
  public void periodic() {
    super.periodic();
    for (int i = 0; i < followerConfigs.length; i++) {
      followerIOs[i].updateInputs(followerInputs[i]);
      Logger.processInputs(getName() + "/" + followerConfigs[i].config.name, followerInputs[i]);
    }
  }
}
