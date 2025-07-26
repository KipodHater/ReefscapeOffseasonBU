package frc.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean motorConnected = false;
    public boolean followerConnected = false;

    public double positionDeg = 0; // degrees
    public double velocityDegPerSec = 0; // degrees/second
    public double motorVoltage = 0; // volts
    public double followerVoltage = 0; // volts
    public double motorTemp = 0; // celsius
    public double followerTemp = 0; // celsius
    public double motorCurrent = 0; // amps
    public double followerCurrent = 0; // amps
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void runVoltage(double voltage) {}

  public default void stop() {}

  public default void runPosition(double position, double feedforward) {}

  public default void setPID(double KP, double KI, double KD) {}

  public default void setConstraints(Constraints constraints) {}

  public default void setBrakeMode(boolean isBrake) {}
}
