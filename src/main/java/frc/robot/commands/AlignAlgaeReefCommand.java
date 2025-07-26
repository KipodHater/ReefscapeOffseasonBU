// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;

public class AlignAlgaeReefCommand extends SequentialCommandGroup {

  public AlignAlgaeReefCommand(
      Drive drive,
      Arm arm,
      Elevator elevator,
      Gripper gripper,
      boolean l2,
      BooleanSupplier gripperHasPiece,
      BooleanSupplier ignoreGripperSensor) {

    RobotState.getInstance().setUpIntakeTargetAlgae();
    addRequirements(drive, arm, elevator, gripper);

    // if (!gripperHasPiece || ignoreGripperSensor) {
    //   addCommands(
    //     Commands.runOnce(
    //       () -> 
    //         drive.setStateAutoAlign(
    //           () ->
    //             RobotState.getInstance().getCoralScoringInfo().alignPose())));
    // }

    addCommands(
      Commands.either(
        Commands.parallel(
          SimpleCommands.moveToAlgaeLxCommand(
            arm,
            elevator, 
            l2 ? 2 : 3, 
            RobotState.getInstance().getCoralScoringInfo().backside()
          ),
          Commands.runOnce(
            () -> 
              drive.setStateAutoAlign(
                () -> RobotState.getInstance().getCoralScoringInfo().alignPose()
              )
          )
        ),
        Commands.none(),
          () -> !gripperHasPiece.getAsBoolean() || ignoreGripperSensor.getAsBoolean()
      )
    );
    
  }
}
