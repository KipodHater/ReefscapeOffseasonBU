package frc.robot.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControllerInterface {
    
    public Trigger scoreCoralButton();

    public Trigger scoreEjectAlgaeButton();

    public Trigger openCloseIntakeButton();

    public Trigger intakeAlgaeFloorButton();

    public Trigger l4NetButton();

    public Trigger l3Button();

    public Trigger l2AlgaeHomeButton();

    public Trigger l1ProcessorButton();

    public Trigger resetGyroButton();

    public Trigger purgeIntakeButton();

    public Trigger returnToDefaultButton();

    public Trigger forceInvertedNetButton();

    public Trigger climbButton();

    public DoubleSupplier xVelocityAnalog();

    public DoubleSupplier yVelocityAnalog();

    public DoubleSupplier rotationVelocityAnalog();
}