package frc.robot.subsystems.gripper;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GripperSensorIOInputsAutoLogged extends GripperSensorIO.GripperSensorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("IsConnected", isConnected);
    table.put("HasGamepiece", hasGamepiece);
  }

  @Override
  public void fromLog(LogTable table) {
    isConnected = table.get("IsConnected", isConnected);
    hasGamepiece = table.get("HasGamepiece", hasGamepiece);
  }

  public GripperSensorIOInputsAutoLogged clone() {
    GripperSensorIOInputsAutoLogged copy = new GripperSensorIOInputsAutoLogged();
    copy.isConnected = this.isConnected;
    copy.hasGamepiece = this.hasGamepiece;
    return copy;
  }
}
