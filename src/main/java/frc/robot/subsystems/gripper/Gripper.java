package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.gripper.GripperConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Gripper {

  private final GripperIO io;
  private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();

  // @RequiredArgsConstructor
  public enum GripperStates {
    IDLE,
    OUTTAKE,
    OUTTAKE_STRONG,
    INTAKE,
    HOLD
  };

  @AutoLogOutput(key = "Gripper/Goal")
  private GripperStates goal = GripperStates.IDLE;

  public Gripper(GripperIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Gripper", inputs);

    switch (goal) {
      case IDLE -> io.stop();

      case OUTTAKE -> io.setVoltageOpenLoop(GRIPPER_OUTTAKE_SPEED);

      case OUTTAKE_STRONG -> io.setVoltageOpenLoop(GRIPPER_OUTTAKEFAST_SPEED);

      case INTAKE -> io.setVoltageOpenLoop(GRIPPER_INTAKE_SPEED);

      case HOLD -> io.setVoltageOpenLoop(0.03);

      default -> io.stop();
    }
    ;
  }

  public void setGripperGoal(GripperStates desiredGoal) {
    this.goal = desiredGoal;
  }

  public void testPeriodic() {}

  public void autonomousInit() {
    io.autonomousInit();
  }
}
