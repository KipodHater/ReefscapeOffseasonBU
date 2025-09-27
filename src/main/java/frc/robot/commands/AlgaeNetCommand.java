// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmStates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveStates;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorStates;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.Gripper.GripperStates;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.ledsStates;
import frc.robot.util.AllianceFlipping;
import java.util.function.BooleanSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeNetCommand extends SequentialCommandGroup {
  /** Creates a new AlgaeNetCommand. */
  public AlgaeNetCommand(
      Drive drive,
      Arm arm,
      Elevator elevator,
      Gripper gripper,
      Leds leds,
      BooleanSupplier gripperHasPiece,
      BooleanSupplier ignoreGripperSensor,
      boolean isBackside) {

    addRequirements(drive, arm, elevator, gripper, leds);

    Command isCloseEnough =
        Commands.waitUntil(
            () ->
                (Math.abs(drive.getPose().getX() - AllianceFlipping.applyX(6.7)) < 0.5
                    // && drive.getPose().getX() - AllianceFlipping.applyX(7.5) > -0.5
                    && drive.getPose().getY() < AllianceFlipping.applyY(9)
                    && drive.getPose().getY() > AllianceFlipping.applyY(5.5)));
    // true);

    addCommands(
        Commands.either(
            Commands.sequence(
                Commands.parallel(
                    Commands.runOnce(
                        () -> leds.setState(ledsStates.ALGAE),
                        leds), // TODO: idk what led color this is
                    SimpleCommands.moveToNetCommand(arm, elevator, gripper, isBackside),
                    Commands.runOnce(() -> drive.setState(DriveStates.FIELD_DRIVE), drive)),
                // Commands.print("fuck"),
                isCloseEnough,
                // Commands.waitUntil(() -> false),
                // Commands.print("yaay?"),
                Commands.runOnce(() -> gripper.setState(GripperStates.EJECT_ALGAE), gripper),
                // Commands.print("letsfuckinggo")),
                Commands.runOnce(() -> arm.setState(ArmStates.HOME), arm),
                Commands.waitUntil(() -> arm.isSafeForElevator()),
                Commands.runOnce(() -> elevator.setState(ElevatorStates.DEFAULT), elevator),
                Commands.waitUntil(() -> elevator.atGoal()),
                Commands.runOnce(() -> arm.setState(ArmStates.DEFAULT), arm)),

            // Commands.print(
            //     (drive.getPose().getX() - AllianceFlipping.applyX(7.5) < 0.5)
            //         + " "
            //         + (drive.getPose().getY() < AllianceFlipping.applyY(9))
            //         + " "
            //         + (drive.getPose().getY() > AllianceFlipping.applyY(5.5))
            //         + " "
            //         + drive.getPose().getY()
            //         + " "
            //         + AllianceFlipping.applyY(7.5)),
            // Commands.print("oguirhgo9uiehjgru9e"),
            // Commands.waitUntil(
            //         () ->
            //             // (drive.getPose().getX() - AllianceFlipping.applyX(7.5) <
            // 0.5
            //             //         // && drive.getPose().getX() -
            // AllianceFlipping.applyX(7.5) >
            //             // -0.5
            //             //         && drive.getPose().getY() <
            // AllianceFlipping.applyY(9)
            //             //         && drive.getPose().getY() >
            // AllianceFlipping.applyY(5.5))
            //             //     || true),
            //             true)

            Commands.print("ru dumb?"),
            () -> gripperHasPiece.getAsBoolean() || ignoreGripperSensor.getAsBoolean()));
  }
}
