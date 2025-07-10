package frc.robot.subsystems.conveyor;

public class Conveyor {

  public ConveyorIO io;
  public ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();

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
}