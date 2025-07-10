package frc.robot.subsystems.climb;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimbIOInputsAutoLogged extends ClimbIO.ClimbIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("MotorConnected", motorConnected);
    table.put("MotorVoltage", motorVoltage);
    table.put("PositionDeg", positionDeg);
    table.put("VelocityDegPerSec", velocityDegPerSec);
    table.put("MotorTemp", motorTemp);
  }

  @Override
  public void fromLog(LogTable table) {
    motorConnected = table.get("MotorConnected", motorConnected);
    motorVoltage = table.get("MotorVoltage", motorVoltage);
    positionDeg = table.get("PositionDeg", positionDeg);
    velocityDegPerSec = table.get("VelocityDegPerSec", velocityDegPerSec);
    motorTemp = table.get("MotorTemp", motorTemp);
  }

  public ClimbIOInputsAutoLogged clone() {
    ClimbIOInputsAutoLogged copy = new ClimbIOInputsAutoLogged();
    copy.motorConnected = this.motorConnected;
    copy.motorVoltage = this.motorVoltage;
    copy.positionDeg = this.positionDeg;
    copy.velocityDegPerSec = this.velocityDegPerSec;
    copy.motorTemp = this.motorTemp;
    return copy;
  }
}
