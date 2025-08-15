package frc.robot.subsystems.conveyor;

import static frc.robot.subsystems.conveyor.ConveyorConstants.REVERSE_VOLTAGE;
import static frc.robot.subsystems.conveyor.ConveyorConstants.currentThreshold;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Conveyor extends SubsystemBase {

  public ConveyorIO io;
  public ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();

  private Timer timer = new Timer();

  @AutoLogOutput(key = "Conveyor/currentState")
  public ConveyorStates currentState = ConveyorStates.IDLE;

  public enum ConveyorStates {
    IDLE(0.0),
    INTAKE(12.0);
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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Conveyor", inputs);
    stateMachine();
  }

  public void stateMachine() {
    switch (currentState) {
      case IDLE -> io.runVoltage(ConveyorStates.IDLE.getVoltage());
      case INTAKE -> {
        if (inputs.motorCurrent > currentThreshold && !timer.isRunning()) {
          timer.start();
        }
        if (timer.isRunning() && timer.get() < 0.3) {
          io.runVoltage(REVERSE_VOLTAGE);
        } else {
          timer.stop();
          timer.reset();
          io.runVoltage(ConveyorStates.INTAKE.getVoltage());
        }
      }
      default -> io.runVoltage(0.0);
    }
  }

  public boolean hasCoral() {
    return false;
  }
}
