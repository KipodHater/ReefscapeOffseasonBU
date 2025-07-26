package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmStates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorStates;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.Gripper.GripperStates;
import java.util.function.Supplier;

public class SimpleCommands {

  private static boolean moveToDefault(Arm arm, Elevator elevator) {
    elevator.setState(ElevatorStates.DEFAULT);
    if (elevator.isSafeForArm()) {
      arm.setState(ArmStates.DEFAULT);
    }
    if (arm.atGoal(ArmStates.DEFAULT) && elevator.atGoal(ElevatorStates.DEFAULT)) {
      arm.setState(ArmStates.DEFAULT);
      elevator.setState(ElevatorStates.DEFAULT);
      return true;
    }
    return false;
  }

  public static Command moveToDefaultCommand(Arm arm, Elevator elevator) {
    return new FunctionalCommand(
        () -> {}, // initialize: do nothing
        () -> {}, // execute: do nothing
        interrupted -> {}, // end: optional cleanup
        () -> moveToDefault(arm, elevator), // check if finished
        arm,
        elevator);
    // .withTimeout(5); // can maybe remove this timeout
  }

  private static boolean moveToLx(Arm arm, Elevator elevator, int lx, boolean isBackside) {
    arm.setReefState(lx, isBackside);
    elevator.setReefState(lx);

    return elevator.atGoal() && arm.atGoal();
  }

  public static Command moveToLxCommand(Arm arm, Elevator elevator, int lx, boolean isBackside) {
    return new FunctionalCommand(
        () -> {},
        () -> {},
        interrupted -> {},
        () -> moveToLx(arm, elevator, lx, isBackside), // check if finished
        arm,
        elevator);
  }

  private static boolean moveToLxScore(
      Arm arm, Elevator elevator, Gripper gripper, int lx, boolean isBackside) {
    arm.setScoreReefState(lx, isBackside);
    elevator.setReefState(lx);
    if (arm.atScoreGoal()) gripper.setState(GripperStates.EJECT_CORAL);
    return arm.atGoal();
  }

  public static Command moveToLxScoreCommand(
      Arm arm, Elevator elevator, Gripper gripper, int lx, boolean isBackside) {
    return new FunctionalCommand(
        () -> {},
        () -> {},
        interrupted -> {},
        () -> moveToLxScore(arm, elevator, gripper, lx, isBackside),
        arm,
        elevator);
  }

  public static Command nonStopAutoAlignCommand(Drive drive, Supplier<Pose2d> poseSupplier) {
    return new FunctionalCommand(
        () -> drive.setStateAutoAlign(poseSupplier),
        () -> {},
        interrupted -> {},
        () -> false,
        drive);
  }

  public static Command placeCoralCommandTeleop(
      Elevator elevator, Arm arm, Gripper gripper, Drive drive, int Lx) {
    return new FunctionalCommand(
        () -> {
          elevator.setReefState(Lx);
          arm.setScoreReefState(Lx, RobotState.getInstance().getCoralScoringInfo().backside());
          gripper.setState(GripperStates.IDLE);
        },
        () -> {
          if (arm.atScoreGoal()) {
            gripper.setState(GripperStates.EJECT_CORAL);
            drive.setStateSlowlyForward(RobotState.getInstance().getCoralScoringInfo().backside());
          }
        },
        interrupted -> {},
        () -> arm.atGoal(),
        arm,
        elevator,
        gripper,
        drive);
  }
}
