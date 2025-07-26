package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO {

  private final ProfiledPIDController armPIDController;
  private final SingleJointedArmSim armSim;

  private double appliedVoltage = 0;

  public ArmIOSim() {
    System.out.println("Arm Sim Constructor");
    armPIDController =
        new ProfiledPIDController(GAINS.KP(), GAINS.KI(), GAINS.KD(), ARM_CONSTRAINTS);
    armSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(2), ARM_MOI, 100.0),
            DCMotor.getNEO(2),
            ARM_GEAR_RATIO,
            ARM_LENGTH_METERS,
            MIN_ANGLE * Math.PI / 180.0,
            MAX_ANGLE * Math.PI / 180.0,
            true,
            60 * Math.PI / 180.0,
            0.05,
            0.05);
    System.out.println("Arm Sim Initialized");
  }

  public void updateInputs(ArmIOInputs inputs) {
    armSim.update(Constants.CYCLE_TIME);
    inputs.positionDeg = armSim.getAngleRads() * 180 / Math.PI;
    inputs.velocityDegPerSec = armSim.getVelocityRadPerSec() * 180 / Math.PI;
    inputs.motorVoltage = appliedVoltage;
    inputs.followerVoltage = appliedVoltage;
    inputs.motorTemp = 0;
    inputs.followerTemp = 0;
    inputs.motorConnected = true;
    inputs.followerConnected = true;
  }

  public void runVoltage(double voltage) {
    appliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    armSim.setInputVoltage(appliedVoltage);
  }

  public void stop() {
    runVoltage(0);
  }

  public void runPosition(double position, double feedforward) {
    double output = armPIDController.calculate(armSim.getAngleRads() * 180 / Math.PI, position);
    runVoltage(output + feedforward);
  }

  public void setPID(double KP, double KI, double KD) {
    armPIDController.setPID(KP, KI, KD);
  }

  public void setConstraints(Constraints constraints) {
    armPIDController.setConstraints(constraints);
  }

  public void setBrakeMode(boolean isBrake) {}
}
