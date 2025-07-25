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

public class AlignCoralCommand extends SequentialCommandGroup {

  public AlignCoralCommand(
      Drive drive,
      Arm arm,
      Elevator elevator,
      Gripper gripper,
      int lx,
      BooleanSupplier gripperHasCoral,
      BooleanSupplier ignoreGripperSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    RobotState.getInstance().setUpScoringTargetCoral();
    addRequirements(drive, arm, elevator, gripper);

    if (!gripperHasCoral.getAsBoolean() || ignoreGripperSensor.getAsBoolean()) {
      addCommands(
          Commands.parallel(
              new IntakeFromConveyor(arm, elevator, gripper, gripperHasCoral, ignoreGripperSensor),
              Commands.runOnce(
                  () ->
                      drive.setStateAutoAlign(
                          () -> RobotState.getInstance().getCoralScoringInfo().alignPose()))));
    }
    addCommands(
        Commands.either(
            Commands.sequence(
                Commands.parallel(
                    SimpleCommands.moveToLxCommand(
                        arm,
                        elevator,
                        lx,
                        RobotState.getInstance().getCoralScoringInfo().backside()),
                    Commands.runOnce(
                        () ->
                            drive.setStateAutoAlign(
                                () -> RobotState.getInstance().getCoralScoringInfo().alignPose()))),
                SimpleCommands.nonStopAutoAlignCommand(drive, () -> RobotState.getInstance().getCoralScoringInfo().scorePose())
                ),
            Commands.none(),
            () -> (gripperHasCoral.getAsBoolean() || ignoreGripperSensor.getAsBoolean())));
  }
}
