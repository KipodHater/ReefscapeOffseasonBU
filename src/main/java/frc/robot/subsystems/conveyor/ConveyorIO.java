package frc.robot.subsystems.conveyor;

import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO {

  @AutoLog
  public static class ConveyorIOInputs {
    public boolean motorConnected = false;
    public boolean followerConnected = false;

    public double motorVelocityDegPerSec = 0.0; // degrees/s
    public double followerVelocityDegPerSec = 0.0; // degrees/s
    public double motorVoltage = 0.0; // volts
    public double followerVoltage = 0.0; // volts
    public double motorTemp = 0.0; // celsius
    public double followerTemp = 0.0; // celsius
    public double motorCurrent = 0.0; // amps
    public double followerCurrent = 0.0; // amps
  }

  public default void updateInputs(ConveyorIOInputs inputs) {}

  public default void runVoltage(double voltage) {}

  public default void stop() {}

  // public default void setPID(double KP, double KI, double KD) {}

  public default void setBrakeMode(boolean isBrake) {}
}
