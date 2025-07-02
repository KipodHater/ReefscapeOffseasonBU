package frc.robot.subsystems.gripper;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GripperIOInputsAutoLogged extends GripperIO.GripperIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("MotorConnected", motorConnected);
    table.put("GripperMotorRPM", gripperMotorRPM);
    table.put("GripperMotorVoltage", gripperMotorVoltage);
    table.put("GripperMotorTemp", gripperMotorTemp);
  }

  @Override
  public void fromLog(LogTable table) {
    motorConnected = table.get("MotorConnected", motorConnected);
    gripperMotorRPM = table.get("GripperMotorRPM", gripperMotorRPM);
    gripperMotorVoltage = table.get("GripperMotorVoltage", gripperMotorVoltage);
    gripperMotorTemp = table.get("GripperMotorTemp", gripperMotorTemp);
  }

  public GripperIOInputsAutoLogged clone() {
    GripperIOInputsAutoLogged copy = new GripperIOInputsAutoLogged();
    copy.motorConnected = this.motorConnected;
    copy.gripperMotorRPM = this.gripperMotorRPM;
    copy.gripperMotorVoltage = this.gripperMotorVoltage;
    copy.gripperMotorTemp = this.gripperMotorTemp;
    return copy;
  }
}
