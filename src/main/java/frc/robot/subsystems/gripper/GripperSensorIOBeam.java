package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.gripper.GripperConstants.BEAMBRAKE_ID;

import edu.wpi.first.wpilibj.DigitalInput;

public class GripperSensorIOBeam implements GripperSensorIO {

  private final DigitalInput beamBreak; // TODO: set channel

  public GripperSensorIOBeam() {
    beamBreak = new DigitalInput(BEAMBRAKE_ID);
  }

  public void updateInputs(GripperSensorIOInputs inputs) {
    inputs.hasGamepiece = !beamBreak.get();
  }
}
