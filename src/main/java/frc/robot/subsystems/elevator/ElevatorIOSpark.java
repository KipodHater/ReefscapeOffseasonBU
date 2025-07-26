package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.sparkStickyFault;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import org.littletonrobotics.junction.AutoLogOutput;

public class ElevatorIOSpark implements ElevatorIO {

  private final SparkMax motor;
  private final SparkMax follower;
  private final SparkMaxConfig config;
  private final SparkMaxConfig followerConfig;

  private final AbsoluteEncoder elevatorEncoder;

  private final ProfiledPIDController elevatorPIDController;

  private boolean brakeEnabled = ELEVATOR_BRAKE;

  private final Debouncer motorDebouncer = new Debouncer(0.5);
  private final Debouncer followerDebouncer = new Debouncer(0.5);

  @AutoLogOutput(key = "Elevator/Setpoint")
  private Double ElevatorSetpoint;

  public ElevatorIOSpark() {
    motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    follower = new SparkMax(FOLLOWER_ID, MotorType.kBrushless);
    elevatorEncoder = motor.getAbsoluteEncoder();

    config = new SparkMaxConfig();
    followerConfig = new SparkMaxConfig();

    config.inverted(ELEVATOR_INVERTED);
    config.idleMode(ELEVATOR_BRAKE ? IdleMode.kBrake : IdleMode.kCoast);

    config.smartCurrentLimit(ELEVATOR_CURRENT_LIMIT).voltageCompensation(12);

    followerConfig.apply(config);
    followerConfig.follow(motor, true);

    config
        .encoder
        .positionConversionFactor(POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    follower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorPIDController =
        new ProfiledPIDController(GAINS.KP(), GAINS.KI(), GAINS.KD(), ELEVATOR_CONSTRAINTS);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sparkStickyFault = false;

    inputs.motorConnected = motorDebouncer.calculate(sparkStickyFault);
    inputs.followerConnected = followerDebouncer.calculate(sparkStickyFault);

    inputs.positionMeters =
        elevatorEncoder.getPosition()
            * 2
            * Math.PI
            * PULLEY_SOURCE_RADIUS
            * ELEVATOR_GEAR_RATIO
            * 8;
    inputs.velocityMPS =
        elevatorEncoder.getVelocity()
            * ELEVATOR_GEAR_RATIO
            * 8
            / 60
            * 2
            * Math.PI
            * PULLEY_SOURCE_RADIUS; // Convert to meters per second
    inputs.motorVoltage = motor.getBusVoltage();
    inputs.followerVoltage = motor.getBusVoltage();
    inputs.motorTemp = motor.getMotorTemperature();
    inputs.followerTemp = follower.getMotorTemperature();
    inputs.motorCurrent = motor.getOutputCurrent();
    inputs.followerCurrent = follower.getOutputCurrent();
  }

  @Override
  public void runVoltage(double voltage) {
    double appliedVoltage = MathUtil.clamp(voltage, -12, 12);
    motor.setVoltage(appliedVoltage);
    follower.setVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    runVoltage(0);
  }

  @Override
  public void runPosition(double position, double feedforward) {
    runVoltage(
        feedforward + elevatorPIDController.calculate(elevatorEncoder.getPosition(), position));
  }

  @Override
  public void setPID(double KP, double KI, double KD) {
    elevatorPIDController.setPID(KP, KI, KD);
  }

  @Override
  public void setConstraints(Constraints constraints) {
    elevatorPIDController.setConstraints(constraints);
  }

  @Override
  public void setBrakeMode(boolean isBrake) {
    brakeEnabled = isBrake;
    config.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
    followerConfig.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    follower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getRotationsToMeter(double rotations) {
    return METERS_PER_ROTATION * rotations;
  }

  public double getMetersToRotations(double meters) {
    return meters * ROTATIONS_PER_METER;
  }

  public double getCurrentMotorRotations() {
    return getMetersToRotations(elevatorEncoder.getPosition());
  }

  @Override
  public void runPositionMeters(double heightMeters, double feedforward) {

    double setpoint = getMetersToRotations(heightMeters); // deg

    double currentpos = getCurrentMotorRotations(); // deg

    double output = elevatorPIDController.calculate(currentpos, setpoint);

    runVoltage(output + feedforward);
  }

  @Override
  public void runExtendPercent(double percent, double feedforward) {
    percent = MathUtil.clamp(percent, -1, 1);
    runPositionMeters(ELEVATOR_LENGTH_METERS * percent, feedforward);
  }
}
