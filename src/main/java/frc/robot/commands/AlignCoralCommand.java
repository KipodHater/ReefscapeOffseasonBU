// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.Gripper.GripperStates;

public class AlignCoralCommand extends SequentialCommandGroup {

  public AlignCoralCommand(Drive drive, Arm arm, Elevator elevator, Gripper gripper, int lx) {
    // Use addRequirements() here to declare subsystem dependencies.
    RobotState.getInstance().setUpScoringTargetCoral();
    addRequirements(drive, arm, elevator, gripper);
    SmartDashboard.putBoolean("Structure/got here", true);
    if (!gripper.hasCoral() || gripper.shouldIgnoreSensor()) {
      addCommands(
          Commands.parallel(
              new IntakeFromConveyor(arm, elevator, gripper),
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
                    Commands.runOnce(() -> gripper.setState(GripperStates.HOLD_CORAL)),
                    SimpleCommands.driveAutoAlignTolerance(
                        drive,
                        () -> RobotState.getInstance().getCoralScoringInfo().scorePose(),
                        0.05,
                        3)),
                Commands.race(
                    SimpleCommands.nonStopAutoAlignCommand(
                        drive, () -> RobotState.getInstance().getCoralScoringInfo().alignPose()),
                    Commands.waitSeconds(0.3)),
                SimpleCommands.nonStopAutoAlignCommand(
                    drive, () -> RobotState.getInstance().getCoralScoringInfo().scorePose())),
            Commands.none(),
            () -> (gripper.hasCoral() || gripper.shouldIgnoreSensor())));
  }
}
