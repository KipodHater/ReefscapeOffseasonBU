package frc.robot.subsystems.gripper;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.gripper.GripperConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class GripperIOSim implements GripperIO {

  private FlywheelSim flywheeelSim;
  // private DCMotor gearBox;
  private PIDController flywheelPIDController;
  private SimpleMotorFeedforward ffController;
  private Double rpmVelocitySetpoint;

  private Double appliedVoltage;

  private final IntakeSimulation intakeSimulation;
  private final SwerveDriveSimulation driveSimulation;

  public GripperIOSim(SwerveDriveSimulation driveSimulation) {
    appliedVoltage = 0.0;

    flywheeelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), GRIPPER_MOMENT_OF_INNERTIA, GEAR_RATIO),
            DCMotor.getNEO(1));

    flywheelPIDController = new PIDController(GAINS.KP(), GAINS.KI(), GAINS.KD());
    ffController = new SimpleMotorFeedforward(GAINS.KS(), GAINS.KV(), GAINS.KA());

    this.driveSimulation = driveSimulation;

    intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Coral", driveSimulation, Centimeter.of(35), Centimeter.of(15), IntakeSide.FRONT, 1);
  }

  @Override
  public void updateInputs(GripperIOInputs inputs) {
    flywheeelSim.update(CYCLE_TIME);

    inputs.gripperMotorRPM = flywheeelSim.getAngularVelocityRPM();
    inputs.gripperMotorVoltage = appliedVoltage;
    inputs.motorConnected = true;
  }

  @Override
  public void setSpeedRPM(double speed) {
    rpmVelocitySetpoint = speed;
    double ffVoltage = ffController.calculate(flywheeelSim.getAngularVelocityRPM());

    setVoltageOpenLoop(
        ffVoltage + flywheelPIDController.calculate(flywheeelSim.getAngularVelocityRPM()));

    runSimulationCondition(speed);
  }

  @Override
  public void setVoltageOpenLoop(double voltage) {
    rpmVelocitySetpoint = null;
    appliedVoltage = MathUtil.clamp(voltage, -12, 12);
    flywheeelSim.setInputVoltage(appliedVoltage);
    runSimulationCondition(voltage);
  }

  @Override
  public void stop() {
    intakeSimulation.stopIntake();
    setVoltageOpenLoop(0);
  }

  public void setPID(double KP, double KI, double KD) {
    flywheelPIDController = new PIDController(KP, KI, KD);
  }

  public void setFF(double KS, double KV, double KA) {
    ffController = new SimpleMotorFeedforward(KS, KV, KA);
  }

  private boolean isNoteInsideIntake() {
    return intakeSimulation.getGamePiecesAmount()
        != 0; // True if there is a game piece in the intake
  }

  private void runSimulationCondition(double speed) {
    if (speed == 0) {
      // System.out.println("Stopping intake");
      intakeSimulation.stopIntake();
    } else if (speed > 0 && !isNoteInsideIntake()) {
      // System.out.println("Starting intake");
      intakeSimulation.startIntake();
      if (DriverStation.isAutonomous()) {
        intakeSimulation.addGamePieceToIntake();
      }
    } else if (speed < 0 && isNoteInsideIntake()) {
      // System.out.println("Ejecting coral");
      intakeSimulation.obtainGamePieceFromIntake();
      intakeSimulation.stopIntake();
      outtakeCoralSimulation();
    }
  }

  private void outtakeCoralSimulation() {
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new ReefscapeCoralOnFly(
                // Obtain robot position from drive simulation
                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                // The scoring mechanism is installed at (0.46, 0) (meters) on the
                // robot
                new Translation2d(0.35, 0),
                // Obtain robot speed from drive simulation
                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                // Obtain robot facing from drive simulation
                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                // The height at which the coral is ejected
                Distance.ofRelativeUnits(0.75, Meter),
                // The initial speed ofzz the coral
                LinearVelocity.ofRelativeUnits(1, MetersPerSecond),
                // The coral is ejected at a 35-degree slope
                Angle.ofRelativeUnits(0, Degree)));
  }

  public void autonomousInit() {
    intakeSimulation.addGamePieceToIntake();
  }
}
