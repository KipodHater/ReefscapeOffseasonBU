package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SingleXboxController implements ControllerInterface {

  private final XboxController controller;

  public SingleXboxController() {
    controller = new XboxController(0);
  }

  @Override
  public Trigger scoreButton() {
    return new Trigger(() -> controller.getRawAxis(3) > 0.8);
  }

  @Override
  public Trigger intakeAlgaeReefButton() {
    return new Trigger(() -> controller.getRightBumperButton());
  }

  @Override
  public Trigger openCloseIntakeButton() {
    return new Trigger(() -> controller.getRawAxis(2) > 0.8);
  }

  @Override
  public Trigger intakeAlgaeFloorButton() {
    return new Trigger(() -> controller.getLeftBumperButton());
  }

  @Override
  public Trigger l4NetButton() {
    return new Trigger(() -> controller.getYButton());
  }

  @Override
  public Trigger l3Button() {
    return new Trigger(() -> controller.getXButton());
  }

  @Override
  public Trigger l2AlgaeHomeButton() {
    return new Trigger(() -> controller.getBButton());
  }

  @Override
  public Trigger l1ProcessorButton() {
    return new Trigger(() -> controller.getAButton());
  }

  @Override
  public Trigger resetGyroButton() {
    return new Trigger(() -> controller.getRawButton(8) && controller.getPOV() == 0.0);
  }

  @Override
  public Trigger purgeIntakeButton() {
    return new Trigger(() -> controller.getPOV() == 90.0);
  }

  @Override
  public Trigger returnToDefaultButton() {
    return new Trigger(() -> controller.getPOV() == 180.0);
  }

  // @Override
  // public Trigger forceInvertedNetButton() {
  //   return new Trigger(() -> controller.getPOV() == 270.0);
  // }

  @Override
  public Trigger climbButton() {
    return new Trigger(() -> controller.getRawButton(8) && controller.getRawButton(7));
  }

  @Override
  public double xVelocityAnalog() {
    return controller.getLeftX();
  }

  @Override
  public double yVelocityAnalog() {
    return -controller.getLeftY();
  }

  @Override
  public double rotationVelocityAnalog() {
    return controller.getRightX();
  }
}
