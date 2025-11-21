package frc.lib.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Configuration class for a servo motor subsystem. Includes motor controller settings and position
 * limits.
 *
 * @author Eitan Dror
 * @param name Motor name
 * @param id Motor controller CAN ID
 * @param sparkConfig Spark Max motor controller configuration
 * @param unitToRotorRatio Ratio of rotor to units of this subsystem. Rotor * this should be the
 *     units.
 * @param kMinPositionUnits Minimum allowed position in output units
 * @param kMaxPositionUnits Maximum allowed position in output units
 * @param usingAbsoluteEncoder Whether to use an absolute encoder for position feedback
 * @param absoluteEncoderToRotorRatio Ratio of absolute encoder to rotor position
 * @param momentOfInertia Moment of inertia for simulation purposes
 */
public class MotorSubsystemConfig {
  public String name = "UNAMED";
  public int id = -1;
  public SparkMaxConfig sparkConfig = new SparkMaxConfig();

  public double unitToRotorRatio = 1.0;
  public double kMinPositionUnits = Double.NEGATIVE_INFINITY;
  public double kMaxPositionUnits = Double.POSITIVE_INFINITY;
  public boolean usingAbsoluteEncoder = false;
  public double absoluteEncoderToRotorRatio = 1.0;

  // only for sim
  public double momentOfInertia = 0.5;
}
