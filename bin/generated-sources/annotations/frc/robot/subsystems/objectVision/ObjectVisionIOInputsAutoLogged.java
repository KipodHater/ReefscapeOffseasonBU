package frc.robot.subsystems.objectVision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ObjectVisionIOInputsAutoLogged extends ObjectVisionIO.ObjectVisionIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("Targets", targets);
    table.put("Timestamp", timestamp);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    targets = table.get("Targets", targets);
    timestamp = table.get("Timestamp", timestamp);
  }

  public ObjectVisionIOInputsAutoLogged clone() {
    ObjectVisionIOInputsAutoLogged copy = new ObjectVisionIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.targets = this.targets.clone();
    copy.timestamp = this.timestamp;
    return copy;
  }
}
