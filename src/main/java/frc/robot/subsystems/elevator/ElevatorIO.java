package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public boolean motorConnected = false;
    public boolean followerConnected = false;

    public double positionMeters = 0; // Meters
    public double velocityMPS = 0; // Meters/second
    public double motorVoltage = 0; // volts
    public double followerVoltage = 0; // volts
    public double motorTemp = 0; // celsius
    public double followerTemp = 0; // celsius
    public double motorCurrent = 0; // amps
    public double followerCurrent = 0; // amps
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void runVoltage(double voltage) {}

  public default void stop() {}

  public default void runPosition(double position, double feedforward) {}

  public default void runPositionMeters(double height, double feedforward) {}

  public default void runExtendPercent(double percent, double feedforward) {}

  public default void setPID(double KP, double KI, double KD) {}

  public default void setConstraints(Constraints constraints) {}

  public default void setBrakeMode(boolean isBrake) {}
}
