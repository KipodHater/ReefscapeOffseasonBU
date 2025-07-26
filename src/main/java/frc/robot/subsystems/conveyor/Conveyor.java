package frc.robot.subsystems.conveyor;

public class Conveyor {

  public ConveyorIO io;
  public ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();
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

  public void periodic() {
    io.updateInputs(inputs);

    io.checkCurrent();
  }

  public void stateMachine() {
    switch (currentState) {
      case IDLE:
        io.runVoltage(ConveyorStates.IDLE.getVoltage());
        break;
      case INTAKE:
        io.runVoltage(ConveyorStates.INTAKE.getVoltage());
        break;
      case REVERSE:
        io.runVoltage(ConveyorStates.REVERSE.getVoltage());
        break;
      default:
        io.runVoltage(0.0);
        break;
    }
  }

  public void setConveyorGoal(ConveyorStates wantedState) {
    if (wantedState != null) {
      currentState = wantedState;
    } else {
      currentState = ConveyorStates.IDLE;
    }
  }
}
