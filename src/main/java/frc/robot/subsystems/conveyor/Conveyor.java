package frc.robot.subsystems.conveyor;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

  public ConveyorIO io;
  public ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();

  @AutoLogOutput(key = "Conveyor/currentState")
  public ConveyorStates currentState = ConveyorStates.IDLE;

  public enum ConveyorStates {
    IDLE(0.0),
    INTAKE(12.0),
    REVERSE(-5.0);

    double value;

    ConveyorStates(double value) {
      this.value = value;
    }

    public double getVoltage() {
      return this.value;
    }
  };

  public Conveyor(ConveyorIO io) {
    this.io = io;
  }

  public void setState(ConveyorStates state) {
      currentState = state;
  }

  public boolean hasCoral(){
    return false;
  }
}
