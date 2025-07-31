package frc.robot.subsystems.intakeDeploy;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeDeploy extends SubsystemBase {
  private final IntakeDeployIO io;
  private final IntakeDeployIOInputsAutoLogged inputs = new IntakeDeployIOInputsAutoLogged();

  private final ArmFeedforward feedforwardController =
      new ArmFeedforward(
          IntakeDeployConstants.GAINS.KS(),
          IntakeDeployConstants.GAINS.KG(),
          IntakeDeployConstants.GAINS.KV());

  public enum IntakeDeployStates {
    CLOSED(0.0),
    DEPLOY(90.0),
    RETRACT(0.0),
    IDLE(null);

    Double value;

    IntakeDeployStates(Double value) {
      this.value = value;
    }

    public Double position() {
      return this.value;
    }
  };

  @AutoLogOutput(key = "IntakeDeploy/currentState")
  private IntakeDeployStates currentState = IntakeDeployStates.IDLE;

  public IntakeDeploy(IntakeDeployIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeDeploy/inputs", inputs);

    stateMachine();
  }

  private void stateMachine() {
    double ffVoltage =
        feedforwardController.calculate(
            inputs.positionDeg * (Math.PI / 180.0), inputs.velocityDegPerSec * (Math.PI / 180.0));
    // Convert velocity to radians per second

    switch (currentState) {
      case CLOSED -> io.runPosition(IntakeDeployStates.CLOSED.position(), ffVoltage);

      case DEPLOY -> io.runPosition(IntakeDeployStates.DEPLOY.position(), ffVoltage);

      case RETRACT -> io.runPosition(IntakeDeployStates.RETRACT.position(), ffVoltage);

      case IDLE -> io.stop();

      default -> io.stop();
    }
  }

  public void setState(IntakeDeployStates state) {
    this.currentState = state;
  }

  public void setBrakeMode(boolean isBrake) {
    io.setBrakeMode(isBrake);
  }
}
