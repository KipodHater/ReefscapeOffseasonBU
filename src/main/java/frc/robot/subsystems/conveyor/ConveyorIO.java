package frc.robot.subsystems.conveyor;

import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO {

  @AutoLog
  public static class ConveyorIOInputs {
    public boolean motorConnected = false;

    public double velocityDegPerSec = 0.0; // degrees/s
    public double motorVoltage = 0.0; // volts
    public double motorTemp = 0.0; // celsius
    public double motorCurrent = 0.0; // amps
  }

  public default void updateInputs(ConveyorIOInputs inputs) {}

  public default void runVoltage(double voltage) {}

  public default void stop() {}

  public default void setBangBang(double KP) {}

  // public default void setPID(double KP, double KI, double KD) {}

  public default void setBrakeMode(boolean isBrake) {}

  public default void checkCurrent() {}
}
