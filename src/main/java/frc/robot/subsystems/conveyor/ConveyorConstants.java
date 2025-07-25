package frc.robot.subsystems.conveyor;

public class ConveyorConstants {

  public static int MOTOR_ID = 59;
  public static boolean MOTOR_INVERTED = false;
  public static boolean MOTOR_BRAKE = false;

  public static double VELOCITY_CONVERSION_FACTOR = 360.0 / 60.0; // to degrees per second

  public static double currentThreshold = 40.0; // amps

  public static double KP = 0.1;
}
