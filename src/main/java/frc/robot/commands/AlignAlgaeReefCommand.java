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

        RobotState.getInstance().setUpIntakeTargetAlgae();
        addRequirements(drive, arm, elevator, gripper);

        // if (!gripperHasPiece || ignoreGripperSensor) {
        // addCommands(
        // Commands.runOnce(
        // () ->
        // drive.setStateAutoAlign(
        // () ->
        // RobotState.getInstance().getCoralScoringInfo().alignPose())));
        // }

        addCommands(
                Commands.either(
                        Commands.parallel(
                                SimpleCommands.moveToAlgaeLxCommand(
                                        arm,
                                        elevator,
                                        isL2 ? 2 : 3,
                                        RobotState.getInstance().getCoralScoringInfo().backside()),
                                Commands.sequence(
                                        SimpleCommands.driveAutoAlignTolerance(
                                                drive,
                                                () -> RobotState.getInstance().getCoralScoringInfo().alignPose(),
                                                0.1,
                                                2),
                                        SimpleCommands.driveAutoAlignTolerance(
                                                drive,
                                                () -> RobotState.getInstance().getCoralScoringInfo().scorePose(),
                                                0.1,
                                                2),
                                        // Commands.parallel(

                                        Commands.runOnce(
                                                () -> drive.setStateSlowlyForward(
                                                        RobotState.getInstance().getCoralScoringInfo().backside()),
                                                drive)
                                // Commands.waitSeconds(0.1).andThen(SimpleCommands.moveToHomeCommand(arm,
                                // elevator, gripper))

                                // )
                                )),
                        Commands.none(),
                        () -> !gripperHasPiece.getAsBoolean() || ignoreGripperSensor.getAsBoolean()));
    }
}
