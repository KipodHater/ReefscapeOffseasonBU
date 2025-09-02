package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControllerInterface {

  public Trigger scoreButton();

  public Trigger openCloseIntakeButton();

  public Trigger intakeAlgaeReefButton();

  public Trigger intakeAlgaeFloorButton();

  public Trigger l4NetButton();

  public Trigger l3Button();

  public Trigger l2AlgaeHomeButton();

  public Trigger l1ProcessorButton();

  public Trigger resetGyroButton();

  public Trigger purgeIntakeButton();

  public Trigger returnToDefaultButton();

  // public Trigger forceInvertedNetButton();

  public Trigger climbButton();

  public double xVelocityAnalog();

  public double yVelocityAnalog();

  public double rotationVelocityAnalog();
}
