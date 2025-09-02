package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.gripper.GripperConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {

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

  private boolean isNextPieceCoral = true; // true for coral, false for algae

  @AutoLogOutput(key = "Gripper/hasCoral")
  private boolean hasCoral = false;

  @AutoLogOutput(key = "Gripper/hasAlgae")
  private boolean hasAlgae = false;

  public Gripper(GripperIO io, GripperSensorIO ioSensor) {
    this.io = io;
    this.ioSensor = ioSensor;

    SmartDashboard.putBoolean("Gripper/Ignore Gripper Sensor", false);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Gripper", inputs);
    ioSensor.updateInputs(sensorInputs);
    Logger.processInputs("GripperSensor", sensorInputs);
    if (sensorInputs.hasGamepiece) {
      if (isNextPieceCoral) { // possible future problem here
        hasCoral = true;
        hasAlgae = false;
      } else {
        hasAlgae = true;
        hasCoral = false;
      }
    } else {
      hasCoral = false;
      hasAlgae = false;
    }
    stateMachine();

    wantedState = currentState;
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
    this.currentState = wantedState;
  }

  public void testPeriodic() {}

  public void autonomousInit() {
    io.autonomousInit();
  }

  public void setNextGamepieceCoral(boolean isNextPieceCoral) {
    if (!(hasCoral || hasAlgae)) this.isNextPieceCoral = isNextPieceCoral;
  }

  public boolean hasGamepiece() {
    return sensorInputs.hasGamepiece;
  }

  public boolean hasCoral() {
    return hasCoral;
  }

  public boolean hasAlgae() {
    return hasAlgae;
  }

  public boolean shouldIgnoreSensor() {
    return SmartDashboard.getBoolean("Gripper/Ignore Gripper Sensor", false);
  }
}
