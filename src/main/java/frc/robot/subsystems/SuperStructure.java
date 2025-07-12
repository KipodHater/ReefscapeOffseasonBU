// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class SuperStructure extends SubsystemBase {

  public enum SuperStructureState {
    HOME,
    TRAVEL,
    INTAKE_CORAL_FLOOR,
    PLACE_CORAL,
    INTAKE_ALGAE_REEF,
    INTAKE_ALGAE_FLOOR,
    PLACE_ALGAE_PROCESSOR,
    PLACE_ALGAE_NET,
    OPEN_CLIMB,
    CLIMBED
  }

  @AutoLogOutput (key = "SuperStructure/currentState")
  private SuperStructureState currentState = SuperStructureState.HOME;

  @AutoLogOutput (key = "SuperStructure/wantedState")
  private SuperStructureState wantedState = SuperStructureState.HOME;

  private SuperStructureState previousState = SuperStructureState.HOME;

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

  public SuperStructure(Arm arm, Climb climb, Conveyor conveyor, Dashboard dashboard, Drive drive, Elevator elevator, 
    Gripper gripper, IntakeDeploy intakeDeploy, IntakeRollers intakeRollers, ObjectVision ObjectVision, Vision vision) {
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
    
  }

  private SuperStructureState handleStateTransition(SuperStructureState wantedState) {
    // TODO: add logic with algae in gripper
    previousState = currentState;
    return switch (wantedState) {
      case HOME -> SuperStructureState.HOME;

      case TRAVEL -> SuperStructureState.TRAVEL;

      case INTAKE_CORAL_FLOOR -> {
        if (conveyor.hasCoral() || gripper.hasCoral()) yield previousState;
        else yield SuperStructureState.INTAKE_CORAL_FLOOR;
      }

      case PLACE_CORAL -> {
        if(conveyor.hasCoral() || gripper.hasCoral()) {
          yield SuperStructureState.PLACE_CORAL;
        } else {
          yield previousState;
        }
      }

      case INTAKE_ALGAE_REEF -> SuperStructureState.INTAKE_ALGAE_REEF;

      case INTAKE_ALGAE_FLOOR -> SuperStructureState.INTAKE_ALGAE_FLOOR;

      case PLACE_ALGAE_PROCESSOR -> SuperStructureState.PLACE_ALGAE_PROCESSOR;

      case PLACE_ALGAE_NET -> SuperStructureState.PLACE_ALGAE_NET;

      case OPEN_CLIMB -> SuperStructureState.OPEN_CLIMB;

      case CLIMBED -> SuperStructureState.CLIMBED;
    };
  }

  private void stateMachine() {
    switch (currentState) {
      case HOME -> {}

      case TRAVEL-> travel();

      case INTAKE_CORAL_FLOOR -> intakeCoralFloor();

      case PLACE_CORAL -> {}

      case INTAKE_ALGAE_REEF -> {}

      case INTAKE_ALGAE_FLOOR -> {}

      case PLACE_ALGAE_PROCESSOR -> {}

      case PLACE_ALGAE_NET -> {}

      case OPEN_CLIMB -> {}

      case CLIMBED -> {}

      default -> {}
    }
  }

  private void travel() {
    arm.setState(ArmStates.DEFAULT);
    climb.setState(ClimbStates.IDLE);
    conveyor.setState(ConveyorStates.IDLE);
    drive.setState(DriveStates.FIELD_DRIVE);
    elevator.setElevatorGoal(ElevatorStates.DEFAULT);
    gripper.setState(GripperStates.IDLE);
    intakeDeploy.setState(IntakeDeployStates.CLOSED);
    intakeRollers.setState(IntakeRollersStates.IDLE);
  }

  private void intakeCoralFloor() {
    arm.setState(ArmStates.DEFAULT);
    climb.setState(ClimbStates.IDLE);
    conveyor.setState(ConveyorStates.INTAKE);
    drive.setState(DriveStates.ASSISTED_DRIVE);
    elevator.setElevatorGoal(ElevatorStates.DEFAULT);
    gripper.setState(GripperStates.IDLE);
    intakeDeploy.setState(IntakeDeployStates.DEPLOY);
    intakeRollers.setState(IntakeRollersStates.INTAKE);
  }

  private void placeCoral() {
    // advanced logic here!
  } 

  private void intakeAlgaeReef() {
    // advanced logic here!
  }
}
