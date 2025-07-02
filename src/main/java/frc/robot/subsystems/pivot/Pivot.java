package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot {

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private final ArmFeedforward feedforwardController =
      new ArmFeedforward(
          PivotConstants.GAINS.KS(), PivotConstants.GAINS.KG(), PivotConstants.GAINS.KV());

  // @RequiredArgsConstructor
  public enum PivotStates {
    DOWN_INTAKE,
    MIDDLE_OUTTAKE,
    UP_INTAKE,
    IDLE
  }

  @AutoLogOutput(key = "Pivot/Goal")
  private PivotStates goal = PivotStates.IDLE;

  public Pivot(PivotIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    double ffVoltage =
        feedforwardController.calculate(
            inputs.positionDeg * Math.PI / 180.0, inputs.velocityDegPerSec * Math.PI / 180.0);

    switch (goal) {
      case DOWN_INTAKE:
        io.runPosition(PivotConstants.BOT_ANGLE, ffVoltage);
        break;

      case MIDDLE_OUTTAKE:
        io.runPosition(PivotConstants.MID_ANGLE, ffVoltage);
        break;

      case UP_INTAKE:
        io.runPosition(PivotConstants.TOP_ANGLE, ffVoltage);
        break;

      default:
        io.stop();
    }
  }

  public void setPivotGoal(PivotStates desiredGoal) {
    goal = desiredGoal;
  }

  public void testPeriodic() {}
}
