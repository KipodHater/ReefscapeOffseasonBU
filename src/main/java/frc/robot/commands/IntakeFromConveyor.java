// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmStates;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorStates;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.Gripper.GripperStates;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeFromConveyor extends Command {

  private enum IntakeFromConveyorStates {
    MOVE_DEFAULT,
    MOVE_TO_CONVEYOR,
    INTAKE_FROM_CONVEYOR,
    FINAL_MOVE_DEFAULT // can maybe not have this
  }

  private IntakeFromConveyorStates currentState = IntakeFromConveyorStates.MOVE_DEFAULT;

  private final Arm arm;
  private final Elevator elevator;
  private final Gripper gripper;
  private boolean isFinished = false;

  private final Debouncer sensorDebouncer = new Debouncer(0.3, DebounceType.kRising);
  private final Timer timer = new Timer();

  public IntakeFromConveyor(Arm arm, Elevator elevator, Gripper gripper) {
    this.arm = arm;
    this.elevator = elevator;
    this.gripper = gripper;
    addRequirements(arm, elevator, gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gripper.setState(GripperStates.IDLE);
    gripper.setNextGamepieceCoral(true);
    currentState = IntakeFromConveyorStates.MOVE_DEFAULT;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (currentState) {
      case MOVE_DEFAULT:
        elevator.setState(ElevatorStates.DEFAULT);
        if (elevator.isSafeForArm()) {
          arm.setState(ArmStates.DEFAULT);
        }
        if (arm.atGoal(ArmStates.DEFAULT) && elevator.atGoal(ElevatorStates.DEFAULT)) {
          arm.setState(ArmStates.DEFAULT);
          elevator.setState(ElevatorStates.DEFAULT);
          currentState = IntakeFromConveyorStates.MOVE_TO_CONVEYOR;
        }
        break;

      case MOVE_TO_CONVEYOR:
        arm.setState(ArmStates.DEFAULT);
        elevator.setState(ElevatorStates.CORAL_INTAKE_CONVEYOR);
        gripper.setState(GripperStates.INTAKE_CORAL);
        if (arm.atGoal() && elevator.atGoal()) {
          timer.reset();
          timer.start();
          currentState = IntakeFromConveyorStates.INTAKE_FROM_CONVEYOR;
        }
        break;

      case INTAKE_FROM_CONVEYOR:
        arm.setState(ArmStates.DEFAULT);
        elevator.setState(ElevatorStates.CORAL_INTAKE_CONVEYOR);
        gripper.setState(GripperStates.INTAKE_CORAL);
        if (sensorDebouncer.calculate(gripper.hasCoral()) && !gripper.shouldIgnoreSensor()) {
          gripper.setState(GripperStates.HOLD_CORAL);
          currentState = IntakeFromConveyorStates.FINAL_MOVE_DEFAULT;
        }
        if (timer.hasElapsed(0.4)) { // can change this constant but i think its fine
          gripper.setState(GripperStates.HOLD_CORAL);
          currentState = IntakeFromConveyorStates.FINAL_MOVE_DEFAULT; // timeout for intake time
        }
        break;

      case FINAL_MOVE_DEFAULT:
        gripper.setState(GripperStates.HOLD_CORAL);
        elevator.setState(ElevatorStates.DEFAULT);
        arm.setState(ArmStates.DEFAULT);
        isFinished = true;
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
