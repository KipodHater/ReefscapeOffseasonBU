package frc.robot.subsystems.gripper;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C.Port;

public class GripperSensorIORev implements GripperSensorIO {
  private final ColorSensorV3 colorSensor;

  public GripperSensorIORev() {
    colorSensor = new ColorSensorV3(Port.kMXP);
  }

  public void updateInputs(GripperSensorIOInputs inputs) {
    inputs.isConnected = colorSensor.isConnected();

    inputs.hasGamepiece = colorSensor.getProximity() > 1200; // TODO: tune
  }
}
