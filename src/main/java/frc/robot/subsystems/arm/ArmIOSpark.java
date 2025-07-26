package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmIOSpark implements ArmIO {

  private final SparkMax motor;
  private final SparkMax follower;
  private final SparkMaxConfig config;
  private final SparkMaxConfig followerConfig;

  private final AbsoluteEncoder armEncoder;

  private final ProfiledPIDController armPIDController;

  private final Debouncer motorDebouncer = new Debouncer(0.5);
  private final Debouncer followerDebouncer = new Debouncer(0.5);

  private boolean brakeEnabled = true;

  @AutoLogOutput(key = "Arm/Setpoint")
  private Double armSetpoint;

  public ArmIOSpark() {
    motor = new SparkMax(ArmConstants.MOTOR_ID, MotorType.kBrushless);
    follower = new SparkMax(ArmConstants.FOLLOWER_ID, MotorType.kBrushless);

    config = new SparkMaxConfig();
    followerConfig = new SparkMaxConfig();

    armEncoder = motor.getAbsoluteEncoder();

    config.inverted(ArmConstants.ARM_INVERTED);

    config.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12.0);

    followerConfig.apply(config);
    followerConfig.follow(motor, true);

    config
        .encoder
        .positionConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(ArmConstants.VELOCITY_CONVERSION_FACTOR)
        .uvwMeasurementPeriod(10);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        follower,
        5,
        () ->
            follower.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    armPIDController =
        new ProfiledPIDController(GAINS.KP(), GAINS.KI(), GAINS.KD(), ARM_CONSTRAINTS);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(
        motor,
        armEncoder::getPosition,
        (position) -> inputs.positionDeg = position > 180 ? position % 360 - 360 : position % 360);
    ifOk(motor, armEncoder::getVelocity, (velocity) -> inputs.velocityDegPerSec = velocity);

    ifOk(motor, motor::getBusVoltage, (voltage) -> inputs.motorVoltage = voltage);
    ifOk(motor, motor::getMotorTemperature, (temp) -> inputs.motorTemp = temp);
    ifOk(motor, motor::getOutputCurrent, (current) -> inputs.motorCurrent = current);

    inputs.motorConnected = motorDebouncer.calculate(sparkStickyFault);

    sparkStickyFault = false;

    ifOk(follower, follower::getBusVoltage, (voltage) -> inputs.followerVoltage = voltage);
    ifOk(follower, follower::getMotorTemperature, (temp) -> inputs.motorTemp = temp);
    ifOk(follower, follower::getOutputCurrent, (current) -> inputs.motorCurrent = current);

    inputs.followerConnected = followerDebouncer.calculate(sparkStickyFault);
  }

  @Override
  public void runVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void stop() {
    runVoltage(0);
  }

  @Override
  public void runPosition(double position, double feedforward) {
    runVoltage(feedforward + armPIDController.calculate(armEncoder.getPosition(), position));
  }

  @Override
  public void setPID(double KP, double KI, double KD) {
    armPIDController.setPID(KP, KI, KD);
  }

  @Override
  public void setConstraints(Constraints constraints) {
    armPIDController.setConstraints(constraints);
  }

  @Override
  public void setBrakeMode(boolean isBrake) {
    if (isBrake == brakeEnabled) {
      return;
    }
    brakeEnabled = isBrake;
    if (isBrake) config.idleMode(IdleMode.kBrake);
    else config.idleMode(IdleMode.kCoast);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }
}
