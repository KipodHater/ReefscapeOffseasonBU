package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Arm extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final LoggedMechanism2d mechanism2d;
  private LoggedMechanismLigament2d armLigament;
  private LoggedMechanismRoot2d armRoot;

  private final ArmFeedforward feedforwardController =
      new ArmFeedforward(ArmConstants.GAINS.KS(), ArmConstants.GAINS.KG(), ArmConstants.GAINS.KV());

  // @RequiredArgsConstructor
  public enum ArmStates {
    DEFAULT(0.0),
    HOME(180.0),
    CORAL_L1(70.0),
    CORAL_L2(100.0),
    CORAL_L3(100.0),
    CORAL_L4(120.0),
    CORAL_L1_SCORE(60.0),
    CORAL_L2_SCORE(90.0),
    CORAL_L3_SCORE(90.0),
    CORAL_L4_SCORE(110.0),
    CORAL_L2_BACK(260.0),
    CORAL_L3_BACK(260.0),
    CORAL_L4_BACK(280.0),
    CORAL_L2_SCORE_BACK(270.0),
    CORAL_L3_SCORE_BACK(270.0),
    CORAL_L4_SCORE_BACK(250.0),
    ALGAE_INTAKE_REEF(90.0),
    ALGAE_INTAKE_LOLIPOP(50.0),
    ALGAE_INTAKE_FLOOR(40.0),
    ALGAE_SCORE_PROCESSOR(70.0),
    ALGAE_SCORE_NET(200.0),
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

    mechanism2d = new LoggedMechanism2d(0.2, 0.2);
    armRoot = mechanism2d.getRoot("Arm", 0, 0);

    armLigament = armRoot.append(new LoggedMechanismLigament2d("arm", 0.8, 0));
    armLigament.setColor(new edu.wpi.first.wpilibj.util.Color8Bit(Color.kAliceBlue));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    armRoot.setPosition(0, RobotState.getInstance().getElevatorOverallHeight());
    armLigament.setAngle(inputs.positionDeg - 90);
    Logger.recordOutput("Arm/Mechanism", mechanism2d);
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

      case CORAL_L2_BACK -> io.runPosition(ArmStates.CORAL_L2_BACK.position(), ffVoltage);

      case CORAL_L3_BACK -> io.runPosition(ArmStates.CORAL_L3_BACK.position(), ffVoltage);

      case CORAL_L4_BACK -> io.runPosition(ArmStates.CORAL_L4_BACK.position(), ffVoltage);

      case CORAL_L1_SCORE -> io.runPosition(ArmStates.CORAL_L1_SCORE.position(), ffVoltage);

      case CORAL_L2_SCORE -> io.runPosition(ArmStates.CORAL_L2_SCORE.position(), ffVoltage);

      case CORAL_L3_SCORE -> io.runPosition(ArmStates.CORAL_L3_SCORE.position(), ffVoltage);

      case CORAL_L4_SCORE -> io.runPosition(ArmStates.CORAL_L4_SCORE.position(), ffVoltage);

      case CORAL_L2_SCORE_BACK -> io.runPosition(
          ArmStates.CORAL_L2_SCORE_BACK.position(), ffVoltage);

      case CORAL_L3_SCORE_BACK -> io.runPosition(
          ArmStates.CORAL_L3_SCORE_BACK.position(), ffVoltage);

      case CORAL_L4_SCORE_BACK -> io.runPosition(
          ArmStates.CORAL_L4_SCORE_BACK.position(), ffVoltage);

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

  public void setReefState(int Lx, boolean isBackside) {
    if (isBackside) {
      switch (Lx) {
        case 1 -> setState(ArmStates.CORAL_L1);
        case 2 -> setState(ArmStates.CORAL_L2_BACK);
        case 3 -> setState(ArmStates.CORAL_L3_BACK);
        case 4 -> setState(ArmStates.CORAL_L4_BACK);
      }
    } else {
      switch (Lx) {
        case 1 -> setState(ArmStates.CORAL_L1);
        case 2 -> setState(ArmStates.CORAL_L2);
        case 3 -> setState(ArmStates.CORAL_L3);
        case 4 -> setState(ArmStates.CORAL_L4);
      }
    }
  }

  public void setScoreReefState(int Lx, boolean isBackside) {
    if (isBackside) {
      switch (Lx) {
        case 1 -> setState(ArmStates.CORAL_L1_SCORE);
        case 2 -> setState(ArmStates.CORAL_L2_SCORE);
        case 3 -> setState(ArmStates.CORAL_L3_SCORE);
        case 4 -> setState(ArmStates.CORAL_L4_SCORE);
      }
    } else {
      switch (Lx) {
        case 1 -> setState(ArmStates.CORAL_L1_SCORE);
        case 2 -> setState(ArmStates.CORAL_L2_SCORE);
        case 3 -> setState(ArmStates.CORAL_L3_SCORE);
        case 4 -> setState(ArmStates.CORAL_L4_SCORE);
      }
    }
  }

  public boolean atGoal() {
    return Math.abs(inputs.positionDeg - currentState.position()) < ARM_POSITION_TOLERANCE_DEG;
  }

  public boolean atScoreGoal() {
    return Math.abs(inputs.positionDeg - currentState.position())
        < ARM_SCORE_POSITION_TOLERANCE_DEG;
  }

  public boolean atGoal(ArmStates desiredState) {
    return Math.abs(inputs.positionDeg - desiredState.position()) < ARM_POSITION_TOLERANCE_DEG;
  }

  public boolean isSafeForElevator() {
    return inputs.positionDeg > 90.0 && inputs.positionDeg < 210.0;
  }

  public void testPeriodic() {}
}
