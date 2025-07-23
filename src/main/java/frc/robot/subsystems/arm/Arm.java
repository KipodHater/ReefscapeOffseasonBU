package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.arm.ArmConstants.*;

public class Arm extends SubsystemBase{

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final ArmFeedforward feedforwardController =
      new ArmFeedforward(ArmConstants.GAINS.KS(), ArmConstants.GAINS.KG(), ArmConstants.GAINS.KV());

  // @RequiredArgsConstructor
  public enum ArmStates {
    DEFAULT(-90.0),
    HOME(90.0),
    CORAL_L1(-20.0),
    CORAL_L2(10.0),
    CORAL_L3(10.0),
    CORAL_L4(30.0),
    CORAL_L1_SCORE(-30.0),
    CORAL_L2_SCORE(0.0),
    CORAL_L3_SCORE(0.0),
    CORAL_L4_SCORE(20.0),
    ALGAE_INTAKE_REEF(0.0),
    ALGAE_INTAKE_LOLIPOP(-40.0),
    ALGAE_INTAKE_FLOOR(-50.0),
    ALGAE_SCORE_PROCESSOR(-20.0),
    ALGAE_SCORE_NET(110.0),
    IDLE(null);

    Double value;

    ArmStates(Double value) {
      this.value = value;
    }

    public Double position() {
      return this.value;
    }
  };

  @AutoLogOutput(key = "Arm/currentState")
  private ArmStates currentState = ArmStates.IDLE;

  // @AutoLogOutput(key = "Arm/wantedState")
  // private ArmStates wantedState = ArmStates.IDLE;

  public Arm(ArmIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    stateMachine();
  }

  // public ArmStates handleStateTransition(ArmStates wantedState) {
  //   return switch(wantedState){
  //     case DEFA -> {
  //       yield ArmStates.HOME;
  //     }
  //     case CORAL_L1 -> {
  //       if (inputs.hasCoral) {
  //         yield ArmStates.CORAL_L1;
  //       } else {
  //         yield ArmStates.HOME;
  //       }
  //     }
  //     case CORAL_L23 -> {
  //       if (inputs.hasCoral) {
  //         yield ArmStates.CORAL_L23;
  //       } else {
  //         yield ArmStates.HOME;
  //       }
  //     }
  //     case CORAL_L4 -> {
  //       if (inputs.hasCoral) {
  //         yield ArmStates.CORAL_L4;
  //       } else {
  //         yield ArmStates.HOME;
  //       }
  //     }
  //     default -> {
  //       System.out.println("arm is dumb");
  //     }
  //   };
  // }

  public void stateMachine() {

    double ffVoltage =
        feedforwardController.calculate(
            inputs.positionDeg * Math.PI / 180.0, inputs.velocityDegPerSec * Math.PI / 180.0);

    switch (currentState) {
      case DEFAULT -> io.runPosition(ArmStates.DEFAULT.position(), ffVoltage);

      case CORAL_L1 -> io.runPosition(ArmStates.CORAL_L1.position(), ffVoltage);

      case CORAL_L2 -> io.runPosition(ArmStates.CORAL_L2.position(), ffVoltage);

      case CORAL_L3 -> io.runPosition(ArmStates.CORAL_L3.position(), ffVoltage);

      case CORAL_L4 -> io.runPosition(ArmStates.CORAL_L4.position(), ffVoltage);

      case CORAL_L1_SCORE -> io.runPosition(ArmStates.CORAL_L1_SCORE.position(), ffVoltage);

      case CORAL_L2_SCORE -> io.runPosition(ArmStates.CORAL_L2_SCORE.position(), ffVoltage);

      case CORAL_L3_SCORE -> io.runPosition(ArmStates.CORAL_L3_SCORE.position(), ffVoltage);

      case CORAL_L4_SCORE -> io.runPosition(ArmStates.CORAL_L4_SCORE.position(), ffVoltage);

      case ALGAE_INTAKE_REEF -> io.runPosition(ArmStates.ALGAE_INTAKE_REEF.position(), ffVoltage);

      case ALGAE_INTAKE_LOLIPOP -> io.runPosition(
          ArmStates.ALGAE_INTAKE_LOLIPOP.position(), ffVoltage);

      case ALGAE_INTAKE_FLOOR -> io.runPosition(ArmStates.ALGAE_INTAKE_FLOOR.position(), ffVoltage);

      case ALGAE_SCORE_PROCESSOR -> io.runPosition(
          ArmStates.ALGAE_SCORE_PROCESSOR.position(), ffVoltage);

      case ALGAE_SCORE_NET -> io.runPosition(ArmStates.ALGAE_SCORE_NET.position(), ffVoltage);

      case HOME -> io.runPosition(ArmStates.HOME.position(), ffVoltage);

      case IDLE -> io.stop();
    }
    ;
  }

  public void setState(ArmStates desiredGoal) {
    currentState = desiredGoal;
  }

  public void setReefState(int Lx, boolean isScore){
    if(isScore) {
      switch(Lx) {
        case 1 -> setState(ArmStates.CORAL_L1_SCORE);
        case 2 -> setState(ArmStates.CORAL_L2_SCORE);
        case 3 -> setState(ArmStates.CORAL_L3_SCORE);
        case 4 -> setState(ArmStates.CORAL_L4_SCORE);
      }
    } else {
      switch(Lx) {
        case 1 -> setState(ArmStates.CORAL_L1);
        case 2 -> setState(ArmStates.CORAL_L2);
        case 3 -> setState(ArmStates.CORAL_L3);
        case 4 -> setState(ArmStates.CORAL_L4);
      }
    }
  }

  public boolean atGoal() {
    return Math.abs(inputs.positionDeg - currentState.position()) < ARM_POSITION_TOLERANCE_DEG;
  }

  public boolean atGoal(ArmStates desiredState) {
    return Math.abs(inputs.positionDeg - desiredState.position()) < ARM_POSITION_TOLERANCE_DEG;
  }

  public void testPeriodic() {}
}
