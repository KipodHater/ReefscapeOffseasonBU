package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmStates;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorStates;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.Gripper.GripperStates;

public class SimpleCommands {

  private static boolean moveToDefault(Arm arm, Elevator elevator) {
    elevator.setElevatorGoal(ElevatorStates.DEFAULT);
    if (elevator.isSafeForArm()) {
      arm.setState(ArmStates.DEFAULT);
    }
    if (arm.atGoal(ArmStates.DEFAULT) && elevator.atGoal(ElevatorStates.DEFAULT)) {
      arm.setState(ArmStates.DEFAULT);
      elevator.setElevatorGoal(ElevatorStates.DEFAULT);
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
        () -> moveToLxScore(arm, elevator, gripper, lx, isBackside), // check if finished
        arm,
        elevator);
  }
}
