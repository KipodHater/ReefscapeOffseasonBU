package frc.robot.subsystems.conveyor;

public class ConveyorConstants {

  public static int MOTOR_ID = 59;
  public static boolean MOTOR_INVERTED = false;
  public static boolean MOTOR_BRAKE = false;

  public static int FOLLOWER_ID = 59;
  public static boolean FOLLOWER_INVERTED = false;
  public static boolean FOLLOWER_BRAKE = false;

  public static double VELOCITY_CONVERSION_FACTOR = 360.0 / 60.0; // to degrees per second

  public static double currentThreshold = 40.0; // amps
  public static double REVERSE_VOLTAGE = -3.0;

  public static int DIGITAL_INPUT_CHANNEL = 1;

  public static double KP = 0.1;
}
