package frc.lib.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class MotorSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO>
    extends SubsystemBase {
  protected final T inputs;
  protected final U io;
  protected final boolean usingAbsoluteEncoder;

  protected double positionSetpoint = 0.0;

  protected MotorSubsystemConfig config;

  public MotorSubsystem(T inputs, U io, MotorSubsystemConfig config) {
    this.inputs = inputs;
    this.io = io;
    this.config = config;

    this.usingAbsoluteEncoder = config.usingAbsoluteEncoder;
  }

  @Override
  public void periodic() {
    double timestamp = RobotController.getFPGATime() * 1e-6;
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    Logger.recordOutput(
        getName() + "/latencyPeriodicSeconds", RobotController.getFPGATime() * 1e-6 - timestamp);
  }

  /**
   * @param positionUnits in the subsystem units
   * @param ffVolts
   */
  public void setPositionSetpoint(double positionUnits, double ffVolts) {
    this.positionSetpoint = positionUnits;
    Logger.recordOutput(getName() + "/Requested/SetpointPositionUnits", positionUnits);
    io.setPositionSetpoint(positionUnits, ffVolts);
  }

  public void setVelocitySetpoint(double velocityUnitsPerSecond) {
    Logger.recordOutput(
        getName() + "/Requested/SetpointVelocityUnitsPerSecond", velocityUnitsPerSecond);
    io.setVelocitySetpoint(velocityUnitsPerSecond);
  }

  public void setMaxMotionSetpointPosition(double positionUnits, double ffVolts) {
    positionSetpoint = positionUnits;
    Logger.recordOutput(getName() + "/Requested/MaxMotionSetpointPositionUnits", positionUnits);
    io.setMaxMotionSetpointPosition(positionUnits, ffVolts);
  }

  public void setMaxMotionSetpointVelocity(double velocityUnitsPerSecond) {
    Logger.recordOutput(
        getName() + "/Requested/MaxMotionSetpointVelocityUnitsPerSecond", velocityUnitsPerSecond);
    io.setMaxMotionSetpointVelocity(velocityUnitsPerSecond);
  }

  public void setVoltageOutput(double voltage) {
    Logger.recordOutput(getName() + "/Requested/VoltageOutputVolts", voltage);
    io.setVoltageOutput(voltage);
  }

  public double getPositionSetpoint() {
    return positionSetpoint;
  }

  public void setCurrentPositionAsZero() {
    io.setCurrentPositionAsZero();
  }

  public void setCurrentPosition(double positionUnits) {
    io.setCurrentPosition(positionUnits);
  }

  public void setNeutralMode(boolean isBrake) {
    io.setNeutralMode(isBrake);
  }

  public void setEnableSoftLimit(boolean fwd, boolean rev) {
    io.setEnableSoftLimit(fwd, rev);
  }

  public void setEnableHardLimit(boolean fwd, boolean rev) {
    io.setEnableHardLimit(fwd, rev);
  }
}
