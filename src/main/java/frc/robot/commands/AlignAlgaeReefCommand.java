// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.Gripper.GripperStates;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.ledsStates;
import java.util.function.BooleanSupplier;

public class AlignAlgaeReefCommand extends SequentialCommandGroup {

  public AlignAlgaeReefCommand(
      Drive drive,
      Arm arm,
      Elevator elevator,
      Gripper gripper,
      Leds leds,
      BooleanSupplier gripperHasPiece,
      BooleanSupplier ignoreGripperSensor) {

    RobotState.getInstance().setUpReefAlgae();
    addRequirements(drive, arm, elevator, gripper, leds);
    gripper.setNextGamepieceCoral(false);

    addCommands(
        Commands.either(
            Commands.sequence(
                Commands.parallel(
                    Commands.runOnce(() -> leds.setState(ledsStates.RED), leds),
                    SimpleCommands.moveToAlgaeLxCommand(
                        arm,
                        elevator,
                        RobotState.getInstance().getAlgaeScoringInfo().reefFace() % 2 == 1 ? 2 : 3,
                        RobotState.getInstance().getAlgaeScoringInfo().backside()),
                    SimpleCommands.driveAutoAlignTolerance(
                        drive,
                        () -> RobotState.getInstance().getAlgaeScoringInfo().alignPose(),
                        0.3,
                        5)),
                Commands.runOnce(() -> gripper.setState(GripperStates.INTAKE_ALGAE), gripper),
                SimpleCommands.driveAutoAlignTolerance(
                    drive,
                    () -> RobotState.getInstance().getAlgaeScoringInfo().scorePose(),
                    0.3,
                    3),
                Commands.runOnce(() -> leds.setState(ledsStates.FINISH_SCORE), leds),
                Commands.waitSeconds(0.4),
                Commands.runOnce(() -> gripper.setState(GripperStates.HOLD_ALGAE), gripper),
                Commands.runOnce(() -> leds.setState(ledsStates.RED), leds),
                Commands.runOnce(
                    () ->
                        drive.setStateSlowlyForward(
                            RobotState.getInstance().getAlgaeScoringInfo().backside()),
                    drive),
                Commands.waitSeconds(1),
                Commands.runOnce(() -> leds.setState(ledsStates.OFF), leds)),
            Commands.none(),
            () -> !gripperHasPiece.getAsBoolean() || ignoreGripperSensor.getAsBoolean()));
  }
}
