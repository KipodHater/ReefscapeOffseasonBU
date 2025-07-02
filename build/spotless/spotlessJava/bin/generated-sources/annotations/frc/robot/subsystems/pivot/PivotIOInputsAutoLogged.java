package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PivotIOInputsAutoLogged extends PivotIO.PivotIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("MotorConnected", motorConnected);
    table.put("FollowerConnected", followerConnected);
    table.put("PositionDeg", positionDeg);
    table.put("VelocityDegPerSec", velocityDegPerSec);
    table.put("MotorVoltage", motorVoltage);
    table.put("MotorTemp", motorTemp);
    table.put("MotorCurrent", motorCurrent);
  }

  @Override
  public void fromLog(LogTable table) {
    motorConnected = table.get("MotorConnected", motorConnected);
    followerConnected = table.get("FollowerConnected", followerConnected);
    positionDeg = table.get("PositionDeg", positionDeg);
    velocityDegPerSec = table.get("VelocityDegPerSec", velocityDegPerSec);
    motorVoltage = table.get("MotorVoltage", motorVoltage);
    motorTemp = table.get("MotorTemp", motorTemp);
    motorCurrent = table.get("MotorCurrent", motorCurrent);
  }

  public PivotIOInputsAutoLogged clone() {
    PivotIOInputsAutoLogged copy = new PivotIOInputsAutoLogged();
    copy.motorConnected = this.motorConnected;
    copy.followerConnected = this.followerConnected;
    copy.positionDeg = this.positionDeg;
    copy.velocityDegPerSec = this.velocityDegPerSec;
    copy.motorVoltage = this.motorVoltage;
    copy.motorTemp = this.motorTemp;
    copy.motorCurrent = this.motorCurrent;
    return copy;
  }
}
