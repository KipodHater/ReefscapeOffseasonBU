package frc.robot.subsystems.intakeDeploy;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeDeployIOInputsAutoLogged extends IntakeDeployIO.IntakeDeployIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("MotorConnected", motorConnected);
    table.put("PositionDeg", positionDeg);
    table.put("VelocityDegPerSec", velocityDegPerSec);
    table.put("MotorVoltage", motorVoltage);
    table.put("MotorTemp", motorTemp);
    table.put("MotorCurrent", motorCurrent);
  }

  @Override
  public void fromLog(LogTable table) {
    motorConnected = table.get("MotorConnected", motorConnected);
    positionDeg = table.get("PositionDeg", positionDeg);
    velocityDegPerSec = table.get("VelocityDegPerSec", velocityDegPerSec);
    motorVoltage = table.get("MotorVoltage", motorVoltage);
    motorTemp = table.get("MotorTemp", motorTemp);
    motorCurrent = table.get("MotorCurrent", motorCurrent);
  }

  public IntakeDeployIOInputsAutoLogged clone() {
    IntakeDeployIOInputsAutoLogged copy = new IntakeDeployIOInputsAutoLogged();
    copy.motorConnected = this.motorConnected;
    copy.positionDeg = this.positionDeg;
    copy.velocityDegPerSec = this.velocityDegPerSec;
    copy.motorVoltage = this.motorVoltage;
    copy.motorTemp = this.motorTemp;
    copy.motorCurrent = this.motorCurrent;
    return copy;
  }
}
