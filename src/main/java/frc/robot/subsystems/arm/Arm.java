package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final ArmFeedforward feedforwardController =
      new ArmFeedforward(ArmConstants.GAINS.KS(), ArmConstants.GAINS.KG(), ArmConstants.GAINS.KV());

  // @RequiredArgsConstructor
  public enum ArmStates {
    DOWN_INTAKE,
    MIDDLE_OUTTAKE,
    UP_INTAKE,
    IDLE
  }

  @AutoLogOutput(key = "Arm/Goal")
  private ArmStates goal = ArmStates.IDLE;

  public Arm(ArmIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    double ffVoltage =
        feedforwardController.calculate(
            inputs.positionDeg * Math.PI / 180.0, inputs.velocityDegPerSec * Math.PI / 180.0);

    switch (goal) {
      case DOWN_INTAKE -> io.runPosition(ArmConstants.BOT_ANGLE, ffVoltage);

      case MIDDLE_OUTTAKE -> io.runPosition(ArmConstants.MID_ANGLE, ffVoltage);

      case UP_INTAKE -> io.runPosition(ArmConstants.TOP_ANGLE, ffVoltage);

      default -> io.stop();
    }
  }

  public void setArmGoal(ArmStates desiredGoal) {
    goal = desiredGoal;
  }

  public void testPeriodic() {}
}
