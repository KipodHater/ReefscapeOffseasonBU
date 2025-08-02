package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    boolean motorConnected = false;

    double motorVoltage = 0; // volts
    double positionDeg = 0; // degrees
    double velocityDegPerSec = 0; // degrees/second
    double motorCurrent = 0; // amps
    double motorTemp = 0; // celsius
  }

  public default void updateInputs(ClimbIOInputs inputs) {}
  ;

  public default void runVoltage(double voltage) {}
  ;

  public default void stop() {}
  ;

  public default void runPosition(double position, double feedforward) {}
  ;

  public default void setPID(double KP, double KI, double KD) {}
  ;

  public default void setConstraints(Constraints constraints) {}
  ;

  public default void setBrakeMode(boolean isBrake) {}
  ;

  // public default void setSoftLimits(double forwardLimit, double reverseLimit) {}
  // ;
}
