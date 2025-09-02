package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SimulationController implements ControllerInterface {
  private final GenericHID controller;

  public SimulationController() {
    controller = new GenericHID(0);
  }

  @Override
  public Trigger scoreButton() {
    return new Trigger(() -> controller.getRawButton(5));
  }

  @Override
  public Trigger intakeAlgaeReefButton() {
    return new Trigger(() -> controller.getRawButton(8));
  }

  @Override
  public Trigger openCloseIntakeButton() {
    return new Trigger(() -> controller.getRawButton(6));
  }

  @Override
  public Trigger intakeAlgaeFloorButton() {
    return new Trigger(() -> controller.getRawButton(7));
  }

  @Override
  public Trigger l4NetButton() {
    return new Trigger(() -> controller.getRawButton(4));
  }

  @Override
  public Trigger l3Button() {
    return new Trigger(() -> controller.getRawButton(3));
  }

  @Override
  public Trigger l2AlgaeHomeButton() {
    return new Trigger(() -> controller.getRawButton(2));
  }

  @Override
  public Trigger l1ProcessorButton() {
    return new Trigger(() -> controller.getRawButton(1));
  }

  @Override
  public Trigger resetGyroButton() {
    return new Trigger(() -> controller.getRawButton(8) && controller.getPOV() == 0.0);
  }

  @Override
  public Trigger purgeIntakeButton() {
    return new Trigger(() -> controller.getRawButton(10));
  }

  @Override
  public Trigger returnToDefaultButton() {
    return new Trigger(() -> controller.getRawButton(11));
  }

  // @Override
  // public Trigger forceInvertedNetButton() {
  //   return new Trigger(() -> controller.getPOV() == 270.0);
  // }

  @Override
  public Trigger climbButton() {
    return new Trigger(() -> controller.getRawButton(9));
  }

  @Override
  public double xVelocityAnalog() {
    return controller.getRawAxis(0);
  }

  @Override
  public double yVelocityAnalog() {
    return -controller.getRawAxis(1);
  }

  @Override
  public double rotationVelocityAnalog() {
    return controller.getRawAxis(4);
  }
}
