package frc.robot.subsystems.climb;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimbIOInputsAutoLoggedAutoLogged extends ClimbIO.ClimbIOInputsAutoLogged implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Position", position);
    table.put("Velocity", velocity);
    table.put("Current", current);
    table.put("AtLimit", atLimit);
  }

  @Override
  public void fromLog(LogTable table) {
    position = table.get("Position", position);
    velocity = table.get("Velocity", velocity);
    current = table.get("Current", current);
    atLimit = table.get("AtLimit", atLimit);
  }

  public ClimbIOInputsAutoLoggedAutoLogged clone() {
    ClimbIOInputsAutoLoggedAutoLogged copy = new ClimbIOInputsAutoLoggedAutoLogged();
    copy.position = this.position;
    copy.velocity = this.velocity;
    copy.current = this.current;
    copy.atLimit = this.atLimit;
    return copy;
  }
}
