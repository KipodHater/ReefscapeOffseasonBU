package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

public interface GripperIO {

  @AutoLog
  public static class GripperIOInputs {
    public boolean motorConnected = false;

    public double gripperMotorRPM = 0; // rots/minute
    public double gripperMotorVoltage = 0; // volts
    public double gripperMotorTemp = 0; // celsius
  }

  public default void updateInputs(GripperIOInputs inputs) {}

  public default void setSpeedRPM(double speed) {}

  public default void setVoltageOpenLoop(double voltage) {}

  public default void stop() {}

  public default void setPID(double KP, double KI, double KD) {}

  public default void setFF(double KS, double KV, double KA) {}

  public default void autonomousInit() {}
}
