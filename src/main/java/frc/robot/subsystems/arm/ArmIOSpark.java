package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkAbsoluteEncoder;
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
  private final SparkMaxConfig config;

  private final SparkAbsoluteEncoder armEncoder;

  private final ProfiledPIDController armPIDController;

  private final Debouncer motorDebouncer = new Debouncer(0.5);

  private boolean brakeEnabled = true;

  @AutoLogOutput(key = "Arm/Setpoint")
  private Double armSetpoint;

  public ArmIOSpark() {
    motor = new SparkMax(ArmConstants.MOTOR_ID, MotorType.kBrushless);

    config = new SparkMaxConfig();

    armEncoder = motor.getAbsoluteEncoder();

    config.inverted(ArmConstants.ARM_INVERTED);

    config.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12.0);

    config
        .encoder
        .positionConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(ArmConstants.VELOCITY_CONVERSION_FACTOR)
        .inverted(ArmConstants.ENCODER_INVERTED)
        .uvwMeasurementPeriod(10);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    armPIDController =
        new ProfiledPIDController(GAINS.KP(), GAINS.KI(), GAINS.KD(), ARM_CONSTRAINTS);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(
        motor,
        armEncoder::getPosition,
        (position) ->
            inputs.positionDeg =
                (position - ArmConstants.ARM_ENCODER_OFFSET) > 180
                    ? (position - ArmConstants.ARM_ENCODER_OFFSET) % 360 - 360
                    : (position - ArmConstants.ARM_ENCODER_OFFSET) % 360);
    ifOk(motor, armEncoder::getVelocity, (velocity) -> inputs.velocityDegPerSec = velocity);

    ifOk(motor, motor::getBusVoltage, (voltage) -> inputs.motorVoltage = voltage);
    ifOk(motor, motor::getMotorTemperature, (temp) -> inputs.motorTemp = temp);
    ifOk(motor, motor::getOutputCurrent, (current) -> inputs.motorCurrent = current);

    inputs.motorConnected = motorDebouncer.calculate(sparkStickyFault);
  }

  @Override
  public void runVoltage(double voltage) {
    if (voltage > 12.0) {
      voltage = 12.0;
    } else if (voltage < -12.0) {
      voltage = -12.0;
    }
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
