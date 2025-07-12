package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.gripper.GripperConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Gripper {

  private final GripperIO io;
  private final GripperSensorIO ioSensor;
  private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();
  private final GripperSensorIOInputsAutoLogged sensorInputs =
      new GripperSensorIOInputsAutoLogged();

  // @RequiredArgsConstructor
  public enum GripperStates { // update these values!!!
    IDLE(0.0),
    EJECT_CORAL(1.0),
    EJECT_CORAL_L1(2.0),
    EJECT_ALGAE(2.0),
    INTAKE_CORAL(-1.0),
    INTAKE_ALGAE(-3.0),
    HOLD_CORAL(-0.2),
    HOLD_ALGAE(-0.2);

    double value;

    GripperStates(double value) {
      this.value = value;
    }

    public double getSpeed() {
      return this.value;
    }
  };

  @AutoLogOutput(key = "Gripper/currentState")
  private GripperStates currentState = GripperStates.IDLE;

  @AutoLogOutput(key = "Gripper/wantedState")
  private GripperStates wantedState = GripperStates.IDLE;

  public Gripper(GripperIO io, GripperSensorIO ioSensor) {
    this.io = io;
    this.ioSensor = ioSensor;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Gripper", inputs);

    currentState = handleStateTransition(wantedState);
    stateMachine();

    wantedState = currentState;
  }

  private GripperStates handleStateTransition(GripperStates wantedState) {
    return switch (wantedState) {
      case EJECT_CORAL -> {
        if (sensorInputs.hasGamepiece) {
          yield GripperStates.EJECT_CORAL;
        } else {
          yield GripperStates.IDLE;
        }
      }
      case EJECT_CORAL_L1 -> {
        if (sensorInputs.hasGamepiece) {
          yield GripperStates.EJECT_CORAL_L1;
        } else {
          yield GripperStates.IDLE;
        }
      }
      case EJECT_ALGAE -> {
        if (sensorInputs.hasGamepiece) {
          yield GripperStates.EJECT_ALGAE;
        } else {
          yield GripperStates.IDLE;
        }
      }
      case INTAKE_CORAL -> {
        if (sensorInputs.hasGamepiece) {
          yield GripperStates.HOLD_CORAL;
        } else {
          yield GripperStates.INTAKE_CORAL;
        }
      }
      case INTAKE_ALGAE -> {
        if (sensorInputs.hasGamepiece) {
          yield GripperStates.HOLD_ALGAE;
        } else {
          yield GripperStates.INTAKE_ALGAE;
        }
      }
      case HOLD_CORAL -> {
        if (sensorInputs.hasGamepiece) {
          yield GripperStates.HOLD_CORAL;
        } else {
          yield GripperStates.IDLE;
        }
      }
      case HOLD_ALGAE -> {
        if (sensorInputs.hasGamepiece) {
          yield GripperStates.HOLD_ALGAE;
        } else {
          yield GripperStates.IDLE;
        }
      }
      default -> {
        yield GripperStates.EJECT_CORAL;
      }
    };
  }

  private void stateMachine() {
    switch (currentState) {
      case IDLE -> io.stop();

      case EJECT_CORAL -> io.setSpeedRPM(GripperStates.EJECT_CORAL.getSpeed());

      case EJECT_CORAL_L1 -> io.setSpeedRPM(GripperStates.EJECT_CORAL_L1.getSpeed());

      case EJECT_ALGAE -> io.setSpeedRPM(GripperStates.EJECT_ALGAE.getSpeed());

      case INTAKE_CORAL -> io.setSpeedRPM(GripperStates.INTAKE_CORAL.getSpeed());

      case INTAKE_ALGAE -> io.setSpeedRPM(GripperStates.INTAKE_ALGAE.getSpeed());

      case HOLD_CORAL -> io.setSpeedRPM(GripperStates.HOLD_CORAL.getSpeed());

      case HOLD_ALGAE -> io.setSpeedRPM(GripperStates.HOLD_ALGAE.getSpeed());

      default -> {
        System.out.println("Gripper is really broken");
        io.stop();
      }
    }
    ;
  }

  public void setState(GripperStates wantedState) {
    this.wantedState = wantedState;
  }

  public void testPeriodic() {}

  public void autonomousInit() {
    io.autonomousInit();
  }

  public boolean hasCoral() {
    return false;
  }
}
