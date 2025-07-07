package frc.robot.subsystems.intakeDeploy;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeDeployIO {
  @AutoLog
  public static class IntakeDeployIOInputs {
    public boolean motorConnected = false;

    public double positionDeg = 0;
    public double velocityDegPerSec = 0;
    public double motorVoltage = 0; // volts
    public double motorTemp = 0; // celsius
    public double motorCurrent = 0; // amps
  }

  public default void updateInputs(IntakeDeployIOInputs inputs) {}

  public default void runVoltage(double voltage) {}

  public default void stop() {}

  public default void runPosition(double position, double feedforward) {}

  public default void setPID(double KP, double KI, double KD) {}

  public default void setBrakeMode(boolean isBrake) {}

  public default void setSoftLimits(double forwardLimit, double reverseLimit) {}
}
