package frc.lib.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

public class SimSparkMaxIO extends SparkMaxIO {

  protected DCMotorSim sim;
  private SparkMaxSim sparkMaxSim;
  private Notifier simNotifier = null;
  private double simPeriod = 0.005;
  private Optional<Double> overrideRPS = Optional.empty();
  private Optional<Double> overridePos = Optional.empty();

  // Used to handle mechanisms that wrap.
  private boolean invertVoltage = false;

  protected AtomicReference<Double> lastRotations = new AtomicReference<>((double) 0.0);
  protected AtomicReference<Double> lastRPS = new AtomicReference<>((double) 0.0);

  protected double getSimRatio() {
    return config.unitToRotorRatio;
  }

  public SimSparkMaxIO(MotorSubsystemConfig config) {
    this(
        config,
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1), config.momentOfInertia, 1.0 / config.unitToRotorRatio),
            DCMotor.getNEO(1),
            0.001,
            0.001));
  }

  public SimSparkMaxIO(MotorSubsystemConfig config, DCMotorSim sim) {
    super(config);
    this.sim = sim;

    simNotifier = new Notifier(this::updateSim);
    simNotifier.startPeriodic(simPeriod);
    sparkMaxSim = new SparkMaxSim(motor, DCMotor.getNEO(1));
  }

  public void setInvertVoltage(boolean invert) {
    this.invertVoltage = invert;
  }

  public void updateSim() {
    double simVoltage = motor.getAppliedOutput() * RoboRioSim.getVInVoltage();
    simVoltage *= invertVoltage ? -1 : 1;
    sim.setInput(simVoltage);
    Logger.recordOutput(config.name + "/Sim/SimVoltage", simVoltage);

    sim.update(simPeriod);

    overridePos.ifPresent(aDouble -> sim.setAngle(aDouble));
    overrideRPS.ifPresent(aDouble -> sim.setAngularVelocity(aDouble));

    double simPositionRads = sim.getAngularPositionRad();
    Logger.recordOutput(config.name + "/Sim/SimulatorPositionRadians", simPositionRads);

    // Mutate rotor position
    double rotorPosition = Units.radiansToRotations(simPositionRads) / getSimRatio();
    lastRotations.set(rotorPosition);
    sparkMaxSim.setPosition(rotorPosition);
    Logger.recordOutput(config.name + "/Sim/setRawRotorPosition", rotorPosition);

    // Mutate rotor vel
    double rotorVel = Units.radiansToRotations(sim.getAngularVelocityRadPerSec()) / getSimRatio();
    lastRPS.set(rotorVel);
    sparkMaxSim.setVelocity(rotorVel);
    Logger.recordOutput(
        config.name + "/Sim/SimulatorVelocityRadS", sim.getAngularVelocityRadPerSec());
  }

  /**
   * @param rps in rads/second
   */
  public void overrideRPS(Optional<Double> rps) {
    overrideRPS = rps;
  }

  // This is the position in radians of the mechanism.
  public void overridePosRads(Optional<Double> pos) {
    overridePos = pos;
  }

  public double getIntendedRPS() {
    return lastRPS.get();
  }
}
