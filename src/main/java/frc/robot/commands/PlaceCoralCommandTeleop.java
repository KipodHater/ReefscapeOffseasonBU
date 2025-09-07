package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.ledsStates;

public class PlaceCoralCommandTeleop extends SequentialCommandGroup {

  public PlaceCoralCommandTeleop(
      Arm arm, Elevator elevator, Gripper gripper, Drive drive, Leds leds, int lx) {
    addRequirements(arm, elevator, gripper, drive, leds);
    if (gripper.hasCoral() || gripper.shouldIgnoreSensor()) {
      addCommands(
          Commands.parallel(
              SimpleCommands.moveToLxScoreCommand(
                  arm,
                  elevator,
                  gripper,
                  lx,
                  RobotState.getInstance().getCoralScoringInfo().backside()),
              Commands.runOnce(() -> leds.setState(ledsStates.RED))));
      addCommands(Commands.waitSeconds(0.05)); // can maybe remove this
      addCommands(
          Commands.parallel(
              Commands.runOnce(() -> gripper.setState(Gripper.GripperStates.EJECT_CORAL)),
              Commands.runOnce(
                  () ->
                      drive.setStateSlowlyForward(
                          RobotState.getInstance().getCoralScoringInfo().backside())),
              Commands.runOnce(() -> leds.setState(ledsStates.FINISH_SCORE))),
          Commands.waitSeconds(0.3));
    }
  }
}
