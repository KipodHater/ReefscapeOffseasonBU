// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import org.littletonrobotics.junction.AutoLogOutput;

public class SuperStructure extends SubsystemBase {

  public enum SuperStructureStates {
    HOME,
    TRAVEL,
    INTAKE_CORAL_FLOOR,
    PLACE_CORAL_ALIGN,
    PLACE_CORAL,
    INTAKE_ALGAE_REEF,
    INTAKE_ALGAE_FLOOR,
    PLACE_ALGAE_PROCESSOR,
    PLACE_ALGAE_NET,
    OPEN_CLIMB,
    CLIMBED
  }

  @AutoLogOutput(key = "SuperStructure/currentState")
  private SuperStructureStates currentState = SuperStructureStates.HOME;

  @AutoLogOutput(key = "SuperStructure/wantedState")
  private SuperStructureStates wantedState = SuperStructureStates.HOME;

  private SuperStructureStates previousState = SuperStructureStates.HOME;

  private int scoringL = 4;
  private boolean scoreRightSide = false;

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
  public void periodic() {}

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

      case PLACE_CORAL -> {
        if (conveyor.hasCoral() || gripper.hasCoral()) {
          yield SuperStructureStates.PLACE_CORAL;
        } else {
          yield previousState;
        }
      }

      case INTAKE_ALGAE_REEF -> SuperStructureStates.INTAKE_ALGAE_REEF;

      case INTAKE_ALGAE_FLOOR -> SuperStructureStates.INTAKE_ALGAE_FLOOR;

      case PLACE_ALGAE_PROCESSOR -> SuperStructureStates.PLACE_ALGAE_PROCESSOR;

      case PLACE_ALGAE_NET -> SuperStructureStates.PLACE_ALGAE_NET;

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

    if (conveyor.hasCoral()) {
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
}
