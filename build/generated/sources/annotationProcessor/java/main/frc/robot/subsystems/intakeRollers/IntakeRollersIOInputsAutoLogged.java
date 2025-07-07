package frc.robot.subsystems.intakeRollers;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeRollersIOInputsAutoLogged extends IntakeRollersIO.IntakeRollersIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("MotorConnected", motorConnected);
    table.put("VelocityDegPerSec", velocityDegPerSec);
    table.put("MotorVoltage", motorVoltage);
    table.put("MotorTemp", motorTemp);
    table.put("MotorCurrent", motorCurrent);
  }

  @Override
  public void fromLog(LogTable table) {
    motorConnected = table.get("MotorConnected", motorConnected);
    velocityDegPerSec = table.get("VelocityDegPerSec", velocityDegPerSec);
    motorVoltage = table.get("MotorVoltage", motorVoltage);
    motorTemp = table.get("MotorTemp", motorTemp);
    motorCurrent = table.get("MotorCurrent", motorCurrent);
  }

  public IntakeRollersIOInputsAutoLogged clone() {
    IntakeRollersIOInputsAutoLogged copy = new IntakeRollersIOInputsAutoLogged();
    copy.motorConnected = this.motorConnected;
    copy.velocityDegPerSec = this.velocityDegPerSec;
    copy.motorVoltage = this.motorVoltage;
    copy.motorTemp = this.motorTemp;
    copy.motorCurrent = this.motorCurrent;
    return copy;
  }
}
