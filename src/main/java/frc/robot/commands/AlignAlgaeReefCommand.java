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
import java.util.function.BooleanSupplier;

public class AlignAlgaeReefCommand extends SequentialCommandGroup {

  public AlignAlgaeReefCommand(
      Drive drive,
      Arm arm,
      Elevator elevator,
      Gripper gripper,
      boolean isL2,
      BooleanSupplier gripperHasPiece,
      BooleanSupplier ignoreGripperSensor) {

    RobotState.getInstance().setUpReefAlgae();
    addRequirements(drive, arm, elevator, gripper);

    addCommands(
        Commands.either(
            Commands.parallel(
                SimpleCommands.moveToAlgaeLxCommand(
                    arm,
                    elevator,
                    isL2 ? 2 : 3,
                    RobotState.getInstance().getAlgaeScoringInfo().backside()),
                Commands.sequence(
                    SimpleCommands.driveAutoAlignTolerance(
                        drive,
                        () -> RobotState.getInstance().getAlgaeScoringInfo().alignPose(),
                        0.3,
                        5),
                    SimpleCommands.driveAutoAlignTolerance(
                        drive,
                        () -> RobotState.getInstance().getAlgaeScoringInfo().scorePose(),
                        0.3,
                        5),
                    Commands.runOnce(
                        () ->
                            drive.setStateSlowlyForward(
                                RobotState.getInstance().getAlgaeScoringInfo().backside()),
                        drive))),
            Commands.none(),
            () -> !gripperHasPiece.getAsBoolean() || ignoreGripperSensor.getAsBoolean()));
  }
}
