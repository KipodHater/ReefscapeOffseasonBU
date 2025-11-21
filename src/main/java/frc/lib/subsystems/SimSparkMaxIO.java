package frc.lib.subsystems;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimSparkMaxIO extends SparkMaxIO {

    protected DCMotorSim sim;
    private SparkMaxSim sparkMaxSim; 
    private Notifier simNotifier = null;
    private double lastUpdateTimestamp = 0.0;
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
        this(config, new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), config.momentOfInertia,
1.0/config.unitToRotorRatio),
            DCMotor.getNEO(1), 0.001, 0.001
            ));

    }

    public SimSparkMaxIO(MotorSubsystemConfig config, DCMotorSim sim) {
        super(config);
        this.sim = sim;

        simNotifier = new Notifier(this::updateSim);
        simNotifier.startPeriodic(0.005);
        sparkMaxSim = new SparkMaxSim(motor, sim.getGearbox());
    }

    public void updateSim() {

    }
}
