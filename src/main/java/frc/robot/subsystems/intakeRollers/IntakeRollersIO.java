package frc.robot.subsystems.intakeRollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
  @AutoLog
  public static class IntakeRollersIOInputs {
    public boolean motorConnected = false;

    // public double positionDeg = 0;
    public double velocityDegPerSec = 0;
    public double motorVoltage = 0; // volts
    public double motorTemp = 0; // celsius
    public double motorCurrent = 0; // amps
  }

  public default void updateInputs(IntakeRollersIOInputs inputs) {}

  public default void runVoltage(double voltage) {}

  public default void stop() {}

  // public default void runVelocityRPM(double speed) {}

  // public default void setPID(double KP, double KI, double KD) {}

  // public default void setFF(double KS, double KV, double KA) {}
}
