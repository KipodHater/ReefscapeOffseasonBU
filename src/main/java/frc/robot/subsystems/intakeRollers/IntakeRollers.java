package frc.robot.subsystems.intakeRollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends SubsystemBase {
  private final IntakeRollersIO io;
  private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

  public enum IntakeRollersStates {
    IDLE(0.0),
    INTAKE(12.0),
    EXHAUST(-12.0);

    double value;

    IntakeRollersStates(double value) {
      this.value = value;
    }

    public double getSpeed() {
      return this.value;
    }
  };

  @AutoLogOutput(key = "IntakeRollers/currentState")
  private IntakeRollersStates currentState = IntakeRollersStates.IDLE;

  public IntakeRollers(IntakeRollersIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRollers", inputs);

    stateMachine();
  }

  private void stateMachine() {
    switch (currentState) {
      case IDLE -> io.stop();

      case INTAKE -> io.runVoltage(IntakeRollersStates.INTAKE.getSpeed());

      case EXHAUST -> io.runVoltage(IntakeRollersStates.EXHAUST.getSpeed());

      default -> io.stop();
    }
  }

  public void setState(IntakeRollersStates wantedState) {
    currentState = wantedState;
  }
}
