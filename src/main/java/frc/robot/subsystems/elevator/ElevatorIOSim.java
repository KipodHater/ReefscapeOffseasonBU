package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorIOSim implements ElevatorIO {
  private final ProfiledPIDController elevatorPIDController;
  private final ElevatorSim elevatorSim;
  private final LoggedMechanism2d mechanism2d;
  private final LoggedMechanismLigament2d elevatorMechanism;
  private final LoggedMechanismRoot2d elevatorRoot;

  private double appliedVoltage = 0;

  public ElevatorIOSim() {
    System.out.println("Elevator Sim Constructor");
    elevatorPIDController =
        new ProfiledPIDController(GAINS.KP(), GAINS.KI(), GAINS.KD(), ELEVATOR_CONSTRAINTS);
    elevatorSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(DCMotor.getNEO(2), 4, PULLEY_SOURCE_RADIUS, 50),
            DCMotor.getNEO(2),
            0,
            ELEVATOR_LENGTH_METERS,
            true,
            0.3,
            new double[] {0.001, 0.001});

    mechanism2d = new LoggedMechanism2d(0.2, 1.7);
    elevatorRoot = mechanism2d.getRoot("Elevator", 0, 0);

    elevatorMechanism = elevatorRoot.append(new LoggedMechanismLigament2d("Elevator", 0, 90));

    System.out.println("Elevator Sim Initialized");
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(Constants.CYCLE_TIME);
    inputs.positionMeters = elevatorSim.getPositionMeters();
    inputs.velocityMPS = elevatorSim.getVelocityMetersPerSecond();
    inputs.motorVoltage = appliedVoltage;
    inputs.followerVoltage = appliedVoltage;
    inputs.motorTemp = 0;
    inputs.followerTemp = 0;
    inputs.motorConnected = true;
    inputs.followerConnected = true;

    elevatorMechanism.setLength(inputs.positionMeters);
    Logger.recordOutput("Elevator/Mechanism", mechanism2d);
  }

  public void runVoltage(double voltage) {
    appliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    elevatorSim.setInputVoltage(appliedVoltage);
  }

  public void stop() {
    runVoltage(0);
  }

  public void runPosition(double position, double feedforward) {
    double output =
        elevatorPIDController.calculate(elevatorSim.getPositionMeters() * 180 / Math.PI, position);
    runVoltage(output + feedforward);
  }

  public void setPID(double KP, double KI, double KD) {
    elevatorPIDController.setPID(KP, KI, KD);
  }

  public void setConstraints(Constraints constraints) {
    elevatorPIDController.setConstraints(constraints);
  }

  public void setBrakeMode(boolean isBrake) {}

  // public void runPositionMeters(double height, double feedforward) {
  //   double output = elevatorPIDController.calculate(elevatorSim.getPositionMeters(), height);
  //   System.out.println(output + " " + feedforward);

  //   runVoltage(output + feedforward);
  // }

  // public void runExtendPercent(double percent, double feedforward) {
  //   double position = elevatorSim.getPositionMeters() + (percent * ELEVATOR_LENGTH_METERS);
  //   runPositionMeters(position, feedforward);
  // }
  public double getRotationsToMeter(double rotations) {
    return METERS_PER_ROTATION * rotations;
  }

  public double getMetersToRotations(double meters) {
    return meters * ROTATIONS_PER_METER;
  }

  public double getCurrentMotorRotations() {
    return getMetersToRotations(elevatorSim.getPositionMeters());
  }

  @Override
  public void runPositionMeters(double heightMeters, double feedforward) {

    double setpoint = getMetersToRotations(heightMeters); // deg

    double currentpos = getCurrentMotorRotations(); // deg

    double output = elevatorPIDController.calculate(currentpos, setpoint);

    elevatorSim.setInputVoltage(output + feedforward);
  }

  @Override
  public void runExtendPercent(double percent, double feedforward) {
    percent = MathUtil.clamp(percent, -1, 1);
    runPositionMeters(ELEVATOR_LENGTH_METERS * percent, feedforward);
  }
}
