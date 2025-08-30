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
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.ledsStates;
import frc.robot.subsystems.objectVision.ObjectVision;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.AutoLogOutput;

public class SuperStructure extends SubsystemBase {

  public enum SuperStructureStates {
    HOME,
    TRAVEL,
    INTAKE_CORAL_FLOOR,
    PLACE_CORAL_L1,
    PLACE_CORAL_L2,
    PLACE_CORAL_L3,
    PLACE_CORAL_L4,
    INTAKE_ALGAE_REEF,
    INTAKE_ALGAE_FLOOR,
    ALGAE_PROCESSOR,
    ALGAE_NET,
    ALGAE_HOME,
    OPEN_CLIMB,
    CLIMBED
  }

  @AutoLogOutput(key = "SuperStructure/currentState")
  private SuperStructureStates currentState = SuperStructureStates.TRAVEL;

  @AutoLogOutput(key = "SuperStructure/wantedState")
  private SuperStructureStates wantedState = SuperStructureStates.TRAVEL;

  @AutoLogOutput(key = "SuperStructure/nextState")
  private SuperStructureStates nextState = SuperStructureStates.TRAVEL;

  private SuperStructureStates previousState = SuperStructureStates.TRAVEL;

  @AutoLogOutput(key = "SuperStructure/wantedIntakeState")
  private IntakeDeployStates wantedIntakeState = IntakeDeployStates.IDLE;

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
  private final Leds leds;
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
      Leds leds,
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
    this.leds = leds;
    this.objectVision = ObjectVision;
    this.vision = vision;
  }

  @Override
  public void periodic() {
    previousState = currentState;
    if(!currentCommand.isScheduled()){
      if(nextState != null){
        currentState = nextState;
        nextState = null;
      }
      if(currentState != wantedState){
        nextState = handleStateTransition(wantedState);
        // current command = ?
        currentCommand.schedule();
      }
    }
    if(!currentCommand.isScheduled()) stateMachine();
  }

  private SuperStructureStates handleStateTransition(SuperStructureStates wantedState) {
    // TODO: add logic with algae in gripper

    return switch (wantedState) {
      case HOME -> SuperStructureStates.HOME;

      case TRAVEL -> SuperStructureStates.TRAVEL;

      case INTAKE_CORAL_FLOOR -> {
        if (conveyor.hasCoral() || gripper.hasCoral()) yield previousState;
        else yield SuperStructureStates.INTAKE_CORAL_FLOOR;
      }

      case PLACE_CORAL_L1 -> {
        if (gripper.hasCoral()) {
          yield SuperStructureStates.PLACE_CORAL_L1;
        } else {
          yield SuperStructureStates.TRAVEL;
        }
      }

      case PLACE_CORAL_L2 -> {
        if (gripper.hasCoral()) {
          yield SuperStructureStates.PLACE_CORAL_L2;
        } else {
          yield SuperStructureStates.TRAVEL;
        }
      }

      case PLACE_CORAL_L3 -> {
        if (gripper.hasCoral()) {
          yield SuperStructureStates.PLACE_CORAL_L3;
        } else {
          yield SuperStructureStates.TRAVEL;
        }
      }

      case PLACE_CORAL_L4 -> {
        if (gripper.hasCoral()) {
          yield SuperStructureStates.PLACE_CORAL_L4;
        } else {
          yield SuperStructureStates.TRAVEL;
        }
      }

      case INTAKE_ALGAE_REEF -> {
        if (!gripper.hasAlgae() && !gripper.hasCoral())
          yield SuperStructureStates.INTAKE_ALGAE_REEF;
        else yield previousState;
      }

      case INTAKE_ALGAE_FLOOR -> {
        if (!gripper.hasAlgae() && !gripper.hasCoral())
          yield SuperStructureStates.INTAKE_ALGAE_FLOOR;
        else yield previousState;
      }

      case ALGAE_PROCESSOR -> {
        if (gripper.hasAlgae()) yield SuperStructureStates.ALGAE_PROCESSOR;
        else yield SuperStructureStates.TRAVEL;
      }

      case ALGAE_NET -> {
        if (gripper.hasAlgae()) yield SuperStructureStates.ALGAE_NET;
        else yield SuperStructureStates.TRAVEL;
      }

      case ALGAE_HOME -> {
        if (gripper.hasAlgae()) yield SuperStructureStates.ALGAE_HOME;
        else yield SuperStructureStates.TRAVEL;
      }

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

      case PLACE_CORAL_L1 -> placeCoralLx(1);

      case PLACE_CORAL_L2 -> placeCoralLx(2);

      case PLACE_CORAL_L3 -> placeCoralLx(3);

      case PLACE_CORAL_L4 -> placeCoralLx(4);

      case INTAKE_ALGAE_REEF -> {
        closeIntakeIfPossible();
        if (previousState != currentState) {
          wantedIntakeState = IntakeDeployStates.CLOSED;
          CommandScheduler.getInstance().cancelAll();
          currentCommand =
              new AlignAlgaeReefCommand(
                  drive, arm, elevator, gripper, leds, () -> false, () -> false);
          currentCommand.schedule();
        }
        if (!CommandScheduler.getInstance().isScheduled(currentCommand)) {
          leds.setState(ledsStates.BLUE);
          drive.setDriveState(DriveStates.FIELD_DRIVE);
          gripper.setState(GripperStates.HOLD_ALGAE);
        }
      }

      case INTAKE_ALGAE_FLOOR -> {
        arm.setState(ArmStates.ALGAE_INTAKE_FLOOR);
        climb.setState(ClimbStates.IDLE);
        conveyor.setState(ConveyorStates.IDLE);
        drive.setState(DriveStates.FIELD_DRIVE);
        if (arm.isSafeForElevator()) elevator.setState(ElevatorStates.ALGAE_INTAKE_FLOOR);
        else
          elevator.setState(
              ElevatorStates.IDLE); // TODO: change this to hold current position state

        gripper.setState(GripperStates.IDLE);
        intakeStateMachine();
      }

      case ALGAE_PROCESSOR -> {
        arm.setState(ArmStates.ALGAE_SCORE_PROCESSOR);
        if (arm.isSafeForElevator()) elevator.setState(ElevatorStates.ALGAE_SCORE_PROCESSOR);
        else
          elevator.setState(
              ElevatorStates.IDLE); // TODO: change this to hold current position state
        conveyor.setState(ConveyorStates.IDLE);
        drive.setState(DriveStates.FIELD_DRIVE);
        gripper.setState(GripperStates.HOLD_ALGAE);
        intakeStateMachine();
      }

      case ALGAE_NET -> {
        arm.setState(ArmStates.ALGAE_SCORE_NET);
        elevator.setState(ElevatorStates.ALGAE_SCORE_NET);
        conveyor.setState(ConveyorStates.IDLE);
        drive.setState(DriveStates.FIELD_DRIVE);
        gripper.setState(GripperStates.HOLD_ALGAE);
        intakeStateMachine();
        leds.setState(ledsStates.ALGAE);
      }

      case ALGAE_HOME -> {
        arm.setState(ArmStates.HOME);
        if (arm.isSafeForElevator()) elevator.setState(ElevatorStates.HOME);
        conveyor.setState(ConveyorStates.IDLE);
        drive.setState(DriveStates.FIELD_DRIVE);
        gripper.setState(GripperStates.HOLD_ALGAE);
        intakeStateMachine();
        leds.setState(ledsStates.ALGAE);
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
    closeIntakeIfPossible();
  }

  private void intakeCoralFloor() {
    climb.setState(ClimbStates.IDLE);
    conveyor.setState(ConveyorStates.INTAKE);
    drive.setState(DriveStates.ASSISTED_DRIVE);
    elevator.setState(ElevatorStates.DEFAULT);

    if (elevator.isSafeForArm()) arm.setState(ArmStates.DEFAULT);
    else arm.setState(ArmStates.IDLE); // TODO: change this to hold current position state

    gripper.setState(GripperStates.IDLE);
    openIntakeIfPossible();

    if (conveyor.hasCoral() || gripper.hasCoral() /* ignore gripper || conveyor sensor */) {
      currentState = SuperStructureStates.TRAVEL;
    }
  }

  private void placeCoralAlignLx(int lx) {
    wantedIntakeState = IntakeDeployStates.CLOSED;
    closeIntakeIfPossible();
    if (previousState != currentState) {

      CommandScheduler.getInstance().cancelAll();
      currentCommand =
          new AlignCoralCommand(
              drive,
              arm,
              elevator,
              gripper,
              lx,
              () -> gripper.hasCoral(), /* dashboard.ignoreGripperSensor*/
              null);
    }
    if (!CommandScheduler.getInstance().isScheduled(currentCommand)) {
      leds.setState(ledsStates.BLUE);
    }
  }

  private void placeCoralLx(int lx) {
    wantedIntakeState = IntakeDeployStates.CLOSED;
    closeIntakeIfPossible();
    if (previousState != currentState) {
      CommandScheduler.getInstance().cancelAll();
      currentCommand = SimpleCommands.placeCoralCommandTeleop(elevator, arm, gripper, drive, lx);
      currentCommand.schedule();
    }
    if (!CommandScheduler.getInstance().isScheduled(currentCommand)) {
      wantedIntakeState = IntakeDeployStates.DEPLOY;
      wantedState =
          SuperStructureStates
              .INTAKE_CORAL_FLOOR; // can add here a condition about safe travel if we want
    }
  }

  private void intakeStateMachine() {
    switch (wantedIntakeState) {
      case IDLE -> {
        intakeDeploy.setState(IntakeDeployStates.CLOSED);
        intakeRollers.setState(IntakeRollersStates.IDLE);
      }
      case DEPLOY -> openIntakeIfPossible();
      case CLOSED -> closeIntakeIfPossible();
      default -> System.out.println("intake logic is broken!");
    }
  }

  private void openIntakeIfPossible() {
    if (conveyor.hasCoral()
        || gripper.hasCoral()
        || (currentState
            == SuperStructureStates.CLIMBED)) { // TODO: add ignore sensors logic and intake coral
      intakeDeploy.setState(IntakeDeployStates.CLOSED);
      intakeRollers.setState(IntakeRollersStates.IDLE);
    } else {
      intakeDeploy.setState(IntakeDeployStates.DEPLOY);
      intakeRollers.setState(IntakeRollersStates.INTAKE);
    }
  }

  private void closeIntakeIfPossible() {
    intakeRollers.setState(IntakeRollersStates.IDLE);
    if (intakeRollers.hasCoral()) { // TODO: add ignore intake sensor and intake sensor
      intakeDeploy.setState(IntakeDeployStates.CLOSED);
    } else {
      intakeDeploy.setState(IntakeDeployStates.DEPLOY);
    }
  }

  public void intakeButtonPress() {
    if (currentState == SuperStructureStates.INTAKE_CORAL_FLOOR) {
      wantedState = SuperStructureStates.TRAVEL;
      wantedIntakeState = IntakeDeployStates.CLOSED;
    } else if (currentState == SuperStructureStates.TRAVEL) {
      wantedState = SuperStructureStates.INTAKE_CORAL_FLOOR;
      wantedIntakeState = IntakeDeployStates.DEPLOY;
    } else if (wantedIntakeState == IntakeDeployStates.DEPLOY) {
      wantedIntakeState = IntakeDeployStates.CLOSED;
    } else {
      wantedIntakeState = IntakeDeployStates.DEPLOY;
    }
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
