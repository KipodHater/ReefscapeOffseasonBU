// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AlignAlgaeReefCommand;
import frc.robot.commands.AlignCoralCommand;
import frc.robot.commands.SimpleCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmStates;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbStates;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.Conveyor.ConveyorStates;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveStates;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorStates;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.Gripper.GripperStates;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy.IntakeDeployStates;
import frc.robot.subsystems.intakeRollers.IntakeRollers;
import frc.robot.subsystems.intakeRollers.IntakeRollers.IntakeRollersStates;
import frc.robot.subsystems.objectVision.ObjectVision;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.AutoLogOutput;

public class SuperStructure extends SubsystemBase {

  public enum SuperStructureStates {
    HOME,
    TRAVEL,
    INTAKE_CORAL_FLOOR,
    PLACE_CORAL_ALIGN_L1,
    PLACE_CORAL_L1,
    PLACE_CORAL_ALIGN_L2,
    PLACE_CORAL_L2,
    PLACE_CORAL_ALIGN_L3,
    PLACE_CORAL_L3,
    PLACE_CORAL_ALIGN_L4,
    PLACE_CORAL_L4,
    INTAKE_ALGAE_REEF,
    INTAKE_ALGAE_FLOOR,
    ALGAE_PROCESSOR,
    ALGAE_NET,
    ALGAE_HOME,
    OPEN_CLIMB,
    CLIMBED
  }
  // public enum SuperStructureWantedStates{
  //   HOME,
  //   TRAVEL,
  //   INTAKE_CORAL_FLOOR,
  //   PLACE_CORAL_ALIGN_L4,
  //   PLACE_CORAL_L4,
  //   PLACE_CORAL_ALIGN_L3,
  //   PLACE_CORAL_L3,
  //   PLACE_CORAL_ALIGN_L2,
  //   PLACE_CORAL_L2,
  //   PLACE_CORAL_ALIGN_L1,
  //   PLACE_CORAL_L1,
  //   INTAKE_ALGAE_REEF,
  //   INTAKE_ALGAE_FLOOR,
  //   PLACE_ALGAE_PROCESSOR,
  //   PLACE_ALGAE_NET,
  //   OPEN_CLIMB,
  //   CLIMBED
  // }

  @AutoLogOutput(key = "SuperStructure/currentState")
  private SuperStructureStates currentState = SuperStructureStates.TRAVEL;

  @AutoLogOutput(key = "SuperStructure/wantedState")
  private SuperStructureStates wantedState = SuperStructureStates.TRAVEL;

  private SuperStructureStates previousState = SuperStructureStates.TRAVEL;

  public Command currentCommand = null;

  private final Arm arm;
  private final Climb climb;
  private final Conveyor conveyor;
  private final Dashboard dashboard;
  private final Drive drive;
  private final Elevator elevator;
  private final Gripper gripper;
  private final IntakeDeploy intakeDeploy;
  private final IntakeRollers intakeRollers;
  private final ObjectVision objectVision;
  private final Vision vision;

  public SuperStructure(
      Arm arm,
      Climb climb,
      Conveyor conveyor,
      Dashboard dashboard,
      Drive drive,
      Elevator elevator,
      Gripper gripper,
      IntakeDeploy intakeDeploy,
      IntakeRollers intakeRollers,
      ObjectVision ObjectVision,
      Vision vision) {
    this.arm = arm;
    this.climb = climb;
    this.conveyor = conveyor;
    this.dashboard = dashboard;
    this.drive = drive;
    this.elevator = elevator;
    this.gripper = gripper;
    this.intakeDeploy = intakeDeploy;
    this.intakeRollers = intakeRollers;
    this.objectVision = ObjectVision;
    this.vision = vision;
  }

  @Override
  public void periodic() {
    currentState = handleStateTransition(wantedState);
    stateMachine();
    wantedState = currentState;
  }

  private SuperStructureStates handleStateTransition(SuperStructureStates wantedState) {
    // TODO: add logic with algae in gripper
    previousState = currentState;
    return switch (wantedState) {
      case HOME -> SuperStructureStates.HOME;

      case TRAVEL -> SuperStructureStates.TRAVEL;

      case INTAKE_CORAL_FLOOR -> {
        if (conveyor.hasCoral() || gripper.hasCoral()) yield previousState;
        else yield SuperStructureStates.INTAKE_CORAL_FLOOR;
      }

      case PLACE_CORAL_ALIGN_L1 -> {
        if (conveyor.hasCoral() || gripper.hasCoral()) {
          yield SuperStructureStates.PLACE_CORAL_ALIGN_L1;
        } else {
          yield previousState;
        }
      }

      case INTAKE_ALGAE_REEF -> SuperStructureStates.INTAKE_ALGAE_REEF;

      case INTAKE_ALGAE_FLOOR -> SuperStructureStates.INTAKE_ALGAE_FLOOR;

      case ALGAE_PROCESSOR -> SuperStructureStates.ALGAE_PROCESSOR;

      case ALGAE_NET -> SuperStructureStates.ALGAE_NET;

      case ALGAE_HOME -> SuperStructureStates.ALGAE_HOME;

      case OPEN_CLIMB -> SuperStructureStates.OPEN_CLIMB;

      case CLIMBED -> SuperStructureStates.CLIMBED;

      default -> {
        System.out.println("SuperStructure: Invalid state transition requested: " + wantedState);
        yield currentState;
      }
    };
  }

  private void stateMachine() {
    switch (currentState) {
      case HOME -> {}

      case TRAVEL -> travel();

      case INTAKE_CORAL_FLOOR -> intakeCoralFloor();

      case PLACE_CORAL_ALIGN_L1 -> {
        if (previousState != currentState) {
          CommandScheduler.getInstance().cancelAll();
          currentCommand =
              new AlignCoralCommand(
                  drive,
                  arm,
                  elevator,
                  gripper,
                  1,
                  () -> gripper.hasCoral(), /* dashboard.ignoreGripperSensor*/
                  null);
        }
      }

      case PLACE_CORAL_L1 -> {
        if (previousState != currentState) {
          CommandScheduler.getInstance().cancelAll();
          currentCommand = SimpleCommands.placeCoralCommandTeleop(elevator, arm, gripper, drive, 1);
          currentCommand.schedule();
        }
        if (!CommandScheduler.getInstance().isScheduled(currentCommand)) {
          currentState =
              SuperStructureStates
                  .INTAKE_CORAL_FLOOR; // can add here a condition about safe travel if we want
        }
      }

      case PLACE_CORAL_ALIGN_L2 -> {
        if (previousState != currentState) {
          CommandScheduler.getInstance().cancelAll();
          currentCommand =
              new AlignCoralCommand(
                  drive,
                  arm,
                  elevator,
                  gripper,
                  2,
                  () -> gripper.hasCoral(), /* dashboard.ignoreGripperSensor*/
                  null);
        }
      }

      case PLACE_CORAL_L2 -> {
        if (previousState != currentState) {
          CommandScheduler.getInstance().cancelAll();
          currentCommand = SimpleCommands.placeCoralCommandTeleop(elevator, arm, gripper, drive, 2);
          currentCommand.schedule();
        }
        if (!CommandScheduler.getInstance().isScheduled(currentCommand)) {
          currentState =
              SuperStructureStates
                  .INTAKE_CORAL_FLOOR; // can add here a condition about safe travel if we want
        }
      }

      case PLACE_CORAL_ALIGN_L3 -> {
        if (previousState != currentState) {
          CommandScheduler.getInstance().cancelAll();
          currentCommand =
              new AlignCoralCommand(
                  drive,
                  arm,
                  elevator,
                  gripper,
                  3,
                  () -> gripper.hasCoral(), /* dashboard.ignoreGripperSensor*/
                  null);
        }
      }

      case PLACE_CORAL_L3 -> {
        if (previousState != currentState) {
          CommandScheduler.getInstance().cancelAll();
          currentCommand = SimpleCommands.placeCoralCommandTeleop(elevator, arm, gripper, drive, 3);
          currentCommand.schedule();
        }
        if (!CommandScheduler.getInstance().isScheduled(currentCommand)) {
          currentState =
              SuperStructureStates
                  .INTAKE_CORAL_FLOOR; // can add here a condition about safe travel if we want
        }
      }

      case PLACE_CORAL_ALIGN_L4 -> {
        if (previousState != currentState) {
          CommandScheduler.getInstance().cancelAll();
          currentCommand =
              new AlignCoralCommand(
                  drive,
                  arm,
                  elevator,
                  gripper,
                  4,
                  () -> gripper.hasCoral(), /* dashboard.ignoreGripperSensor*/
                  null);
        }
      }

      case PLACE_CORAL_L4 -> {
        if (previousState != currentState) {
          CommandScheduler.getInstance().cancelAll();
          currentCommand = SimpleCommands.placeCoralCommandTeleop(elevator, arm, gripper, drive, 4);
          currentCommand.schedule();
        }
        if (!CommandScheduler.getInstance().isScheduled(currentCommand)) {
          currentState =
              SuperStructureStates
                  .INTAKE_CORAL_FLOOR; // can add here a condition about safe travel if we want
        }
      }

      case INTAKE_ALGAE_REEF -> {
        if (previousState != currentState) {
          CommandScheduler.getInstance().cancelAll();
          currentCommand =
              new AlignAlgaeReefCommand(
                  drive, arm, elevator, gripper, true, () -> false, () -> false);
          currentCommand.schedule();
        }
        if (!CommandScheduler.getInstance().isScheduled(currentCommand)) {
          currentState = SuperStructureStates.ALGAE_HOME;
        }
      }

      case INTAKE_ALGAE_FLOOR -> {}

      case ALGAE_PROCESSOR -> {}

      case ALGAE_NET -> {}

      case ALGAE_HOME -> {
        if (previousState != currentState) {
          CommandScheduler.getInstance().cancelAll();
          currentCommand = SimpleCommands.moveToHomeCommand(arm, elevator, gripper, false);
          currentCommand.schedule();
        }
        if (!CommandScheduler.getInstance().isScheduled(currentCommand)) {
          currentCommand.cancel();
          currentState = SuperStructureStates.TRAVEL;
        }
      }

      case OPEN_CLIMB -> {}

      case CLIMBED -> {}

      default -> {}
    }
  }

  private void travel() {
    climb.setState(ClimbStates.IDLE);
    conveyor.setState(ConveyorStates.IDLE);
    drive.setState(DriveStates.FIELD_DRIVE);
    elevator.setState(ElevatorStates.DEFAULT);

    if (elevator.isSafeForArm()) arm.setState(ArmStates.DEFAULT);
    else arm.setState(ArmStates.IDLE); // TODO: change this to hold current position state

    gripper.setState(GripperStates.IDLE);
    intakeDeploy.setState(IntakeDeployStates.CLOSED);
    intakeRollers.setState(IntakeRollersStates.IDLE);
  }

  private void intakeCoralFloor() {
    climb.setState(ClimbStates.IDLE);
    conveyor.setState(ConveyorStates.INTAKE);
    drive.setState(DriveStates.ASSISTED_DRIVE);
    elevator.setState(ElevatorStates.DEFAULT);

    if (elevator.isSafeForArm()) arm.setState(ArmStates.DEFAULT);
    else arm.setState(ArmStates.IDLE); // TODO: change this to hold current position state

    gripper.setState(GripperStates.IDLE);
    intakeDeploy.setState(IntakeDeployStates.DEPLOY);
    intakeRollers.setState(IntakeRollersStates.INTAKE);

    if (conveyor.hasCoral() || gripper.hasCoral() /* ignore gripper || conveyor sensor */) {
      currentState = SuperStructureStates.TRAVEL;
    }
  }

  // private void placeCoralAlign(int Lx, boolean shouldScoreRightSide) {
  //   // run place coral command, when finished automatically move to intake floor or travel
  // depended on if safe mode enabled
  //   if(!conveyor.hasCoral() && !gripper.hasCoral()) { // gripper.hasAlgae()
  //     currentState = SuperStructureStates.TRAVEL;
  //     return;
  //   }

  //   if(conveyor.hasCoral() && !gripper.hasCoral()){
  //     arm.setState(ArmStates.DEFAULT);
  //     gripper.setState(GripperStates.INTAKE_CORAL);
  //     if(arm.atGoal()){
  //       elevator.setElevatorGoal(ElevatorStates.CORAL_INTAKE_CONVEYOR);
  //     } else {
  //       elevator.setElevatorGoal(ElevatorStates.DEFAULT);
  //     }
  //   } else if(gripper.hasCoral()){ // hasCoral is redundant but here for clarity
  //     arm.setReefState(Lx, false);
  //     elevator.setReefState(Lx);
  //     if(arm.atGoal() && elevator.atGoal() && swerve.atPose(new Pose2d())) {
  //       currentState = SuperStructureStates.PLACE_CORAL;
  //       wantedState = SuperStructureStates.PLACE_CORAL;
  //     }
  //   }
  // }

  // private void placeCoral(int Lx){
  //   gripper.setState(coast);
  //   arm.setScoreState(Lx, scoreRightSide);
  //   elevator.setReefState(Lx);
  //   // and tell the swerve to slowly go backwords, can even add a timer here
  //   if(!gripper.hasCoral()) currentState = SuperStructureStates.TRAVEL; // or is finished
  // }

  private void intakeAlgaeReef() {
    // choose what reef face
    // now command to take it from reef
    // can even check a box that this reef face is clean if the command is successful
  }

  private void intakeAlgaeFloor() {
    // just move every subsystem to the right place and thats it
    // if has algae then move to default algae
  }

  private void placeAlgaeProcessor() {
    // just move the subsystems to the right place and thats it
    // need a smart way to close the arm again
  }

  private void placeAlgaeNet() {
    // just move the subsystems to the right place and thats it
    // need a smart way to close the arm again
  }

  private void setWantedState(SuperStructureStates wantedState) {
    this.wantedState = wantedState;
  }

  public Command setWantedStateCommand(SuperStructureStates wantedState) {
    return new InstantCommand(() -> setWantedState(wantedState));
  }

  public Command setWantedStateCommand(SuperStructureStates wantedState, int scoreL) {
    return new InstantCommand(() -> setWantedState(wantedState));
  }
}
