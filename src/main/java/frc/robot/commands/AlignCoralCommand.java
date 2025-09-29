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

public class AlignCoralCommand extends SequentialCommandGroup {

  public AlignCoralCommand(
      Arm arm, Drive drive, Elevator elevator, Gripper gripper, Leds leds, int lx) {
    // Use addRequirements() here to declare subsystem dependencies.
    RobotState.getInstance().setUpScoringTargetCoral();
    addRequirements(drive, arm, elevator, leds, gripper);
    if (!gripper.hasCoral() || gripper.shouldIgnoreSensor()) {
      addCommands(
          Commands.parallel(
              Commands.runOnce(() -> leds.setState(Leds.ledsStates.PURPLE), leds),
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
                    Commands.runOnce(() -> leds.setState(Leds.ledsStates.PURPLE), leds),
                    SimpleCommands.moveToLxCommand(
                        arm,
                        elevator,
                        lx,
                        RobotState.getInstance().getCoralScoringInfo().backside()),
                    Commands.runOnce(() -> gripper.setState(GripperStates.HOLD_CORAL)),
                    SimpleCommands.driveAutoAlignTolerance(
                        drive,
                        () -> RobotState.getInstance().getCoralScoringInfo().alignPose(),
                        0.03,
                        2)), // can possibly make this forever
                Commands.parallel(
                    SimpleCommands.nonStopAutoAlignCommand(
                        drive, () -> RobotState.getInstance().getCoralScoringInfo().scorePose())),
                SimpleCommands.blinkLedsOnAlignCondition(
                    leds, () -> drive.isAtAlignSetpoint(0.03, 2))),
            Commands.none(),
            () -> (gripper.hasCoral() || gripper.shouldIgnoreSensor())));
  }
}
