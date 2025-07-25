package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final ElevatorFeedforward feedforwardController =
      new ElevatorFeedforward(
          ElevatorConstants.GAINS.KS(), ElevatorConstants.GAINS.KG(), ElevatorConstants.GAINS.KV());

  public enum ElevatorStates {
    DEFAULT(0.4),
    CORAL_INTAKE_CONVEYOR(0.4),
    CORAL_L1(0.2),
    CORAL_L2(0.3),
    CORAL_L3(0.5),
    CORAL_L4(0.6),
    CORAL_L2_SCORE(0.25),
    CORAL_L3_SCORE(0.45),
    CORAL_L4_SCORE(0.55),
    ALGAE_INTAKE_REEF_L2(0.4),
    ALGAE_INTAKE_REEF_L3(0.6),
    ALGAE_INTAKE_LOLIPOP(0.1),
    ALGAE_INTAKE_FLOOR(0.0),
    ALGAE_SCORE_PROCESSOR(0.0),
    ALGAE_SCORE_NET(1.7),
    IDLE(null);

    Double value;

    ElevatorStates(Double value) {
      this.value = value;
    }

    public Double position() {
      return this.value;
    }
  };

  @AutoLogOutput(key = "Elevator/currentState")
  private ElevatorStates currentState = ElevatorStates.IDLE;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    stateMachine();
  }

  public void stateMachine() {

    double ffVoltage = feedforwardController.calculate(inputs.velocityMPS);

    // for (ElevatorStates state : ElevatorStates.values()) {
    //     if (state == currentState) {
    //         if (state.position() != null) {
    //             double position = state.position();
    //             io.runPositionMeters(position, ffVoltage);
    //         } else {
    //             io.stop();
    //         }
    //         return;
    //     }

    // }

    switch (currentState) {
      case DEFAULT -> io.runPositionMeters(ElevatorStates.DEFAULT.position(), ffVoltage);

      case CORAL_L1 -> io.runPositionMeters(ElevatorStates.CORAL_L1.position(), ffVoltage);

      case CORAL_L2 -> io.runPositionMeters(ElevatorStates.CORAL_L2.position(), ffVoltage);

      case CORAL_L3 -> io.runPositionMeters(ElevatorStates.CORAL_L3.position(), ffVoltage);

      case CORAL_L4 -> io.runPositionMeters(ElevatorStates.CORAL_L4.position(), ffVoltage);

      case CORAL_L2_SCORE -> io.runPositionMeters(
          ElevatorStates.CORAL_L2_SCORE.position(), ffVoltage);

      case CORAL_L3_SCORE -> io.runPositionMeters(
          ElevatorStates.CORAL_L3_SCORE.position(), ffVoltage);

      case CORAL_L4_SCORE -> io.runPositionMeters(
          ElevatorStates.CORAL_L4_SCORE.position(), ffVoltage);

      case ALGAE_INTAKE_REEF_L2 -> io.runPositionMeters(
          ElevatorStates.ALGAE_INTAKE_REEF_L2.position(), ffVoltage);

      case ALGAE_INTAKE_REEF_L3 -> io.runPositionMeters(
          ElevatorStates.ALGAE_INTAKE_REEF_L3.position(), ffVoltage);

      case ALGAE_INTAKE_LOLIPOP -> io.runPositionMeters(
          ElevatorStates.ALGAE_INTAKE_LOLIPOP.position(), ffVoltage);

      case ALGAE_INTAKE_FLOOR -> io.runPositionMeters(
          ElevatorStates.ALGAE_INTAKE_FLOOR.position(), ffVoltage);

      case ALGAE_SCORE_PROCESSOR -> io.runPositionMeters(
          ElevatorStates.ALGAE_SCORE_PROCESSOR.position(), ffVoltage);

      case ALGAE_SCORE_NET -> io.runPositionMeters(
          ElevatorStates.ALGAE_SCORE_NET.position(), ffVoltage);

      case IDLE -> io.stop();
    }
  }

  public void setElevatorGoal(ElevatorStates desiredGoal) {
    currentState = desiredGoal;
  }

  public void setReefState(int Lx) {
    switch (Lx) {
      case 1 -> setElevatorGoal(ElevatorStates.CORAL_L1);
      case 2 -> setElevatorGoal(ElevatorStates.CORAL_L2);
      case 3 -> setElevatorGoal(ElevatorStates.CORAL_L3);
      case 4 -> setElevatorGoal(ElevatorStates.CORAL_L4);
    }
  }

  public boolean atGoal() {
    return Math.abs(inputs.positionMeters - currentState.position())
        < 0.01; // can add here a constant
  }

  public boolean atGoal(ElevatorStates desiredState) {
    return Math.abs(inputs.positionMeters - desiredState.position()) < 0.01; // convert cm to m
  }

  public boolean isSafeForArm() {
    return inputs.positionMeters < SAFE_FOR_ARM_HEIGHT;
  }
}
