package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputsAutoLogged extends ElevatorIO.ElevatorIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("MotorConnected", motorConnected);
    table.put("FollowerConnected", followerConnected);
    table.put("PositionMeters", positionMeters);
    table.put("VelocityMPS", velocityMPS);
    table.put("MotorVoltage", motorVoltage);
    table.put("FollowerVoltage", followerVoltage);
    table.put("MotorTemp", motorTemp);
    table.put("FollowerTemp", followerTemp);
    table.put("MotorCurrent", motorCurrent);
    table.put("FollowerCurrent", followerCurrent);
  }

  @Override
  public void fromLog(LogTable table) {
    motorConnected = table.get("MotorConnected", motorConnected);
    followerConnected = table.get("FollowerConnected", followerConnected);
    positionMeters = table.get("PositionMeters", positionMeters);
    velocityMPS = table.get("VelocityMPS", velocityMPS);
    motorVoltage = table.get("MotorVoltage", motorVoltage);
    followerVoltage = table.get("FollowerVoltage", followerVoltage);
    motorTemp = table.get("MotorTemp", motorTemp);
    followerTemp = table.get("FollowerTemp", followerTemp);
    motorCurrent = table.get("MotorCurrent", motorCurrent);
    followerCurrent = table.get("FollowerCurrent", followerCurrent);
  }

  public ElevatorIOInputsAutoLogged clone() {
    ElevatorIOInputsAutoLogged copy = new ElevatorIOInputsAutoLogged();
    copy.motorConnected = this.motorConnected;
    copy.followerConnected = this.followerConnected;
    copy.positionMeters = this.positionMeters;
    copy.velocityMPS = this.velocityMPS;
    copy.motorVoltage = this.motorVoltage;
    copy.followerVoltage = this.followerVoltage;
    copy.motorTemp = this.motorTemp;
    copy.followerTemp = this.followerTemp;
    copy.motorCurrent = this.motorCurrent;
    copy.followerCurrent = this.followerCurrent;
    return copy;
  }
}
