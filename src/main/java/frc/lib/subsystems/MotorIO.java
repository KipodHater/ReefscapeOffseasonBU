package frc.lib.subsystems;

public interface MotorIO {
  void updateInputs(MotorInputs inputs);

  /**
   * @param positionUnits in subsystem units
   */
  void setPositionSetpoint(double units, double ffVolts);

  /**
   * @param positionUnits in subsystem units per second
   */
  void setVelocitySetpoint(double unitsPerSecond);

  /**
   * @param positionUnits in subsystem units
   */
  void setMaxMotionSetpointPosition(double units, double ffVolts);

  /**
   * @param positionUnits in subsystem units per second
   */
  void setMaxMotionSetpointVelocity(double unitsPerSecond);

  /**
   * @param voltage in volts
   */
  void setVoltageOutput(double voltage);

  /** Sets the current position of the motor as zero position. */
  void setCurrentPositionAsZero();

  /** Sets the current position of the motor as the given position. */
  void setCurrentPosition(double positionUnits);

  void setNeutralMode(boolean isBrake);
  // void follow(int masterID, boolean opposeMaterDirection);

  void setEnableSoftLimit(boolean fwd, boolean rev);

  void setEnableHardLimit(boolean fwd, boolean rev);
}
