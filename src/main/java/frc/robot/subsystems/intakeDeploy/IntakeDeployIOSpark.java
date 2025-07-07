package frc.robot.subsystems.intakeDeploy;

import static frc.robot.subsystems.intakeDeploy.IntakeDeployConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import org.littletonrobotics.junction.AutoLogOutput;

public class IntakeDeployIOSpark implements IntakeDeployIO {
  private final SparkMax motor;
  private final SparkMaxConfig config;

  private final AbsoluteEncoder encoder;

  private final ProfiledPIDController intakePIDController;

  private final Debouncer motorDebouncer = new Debouncer(0.5);

  private boolean brakeEnabled = true;

  @AutoLogOutput(key = "IntakeDeploy/Setpoint")
  private Double intakeSetpoint;

  public IntakeDeployIOSpark() {
    motor = new SparkMax(MOTOR_ID, SparkMax.MotorType.kBrushless);
    config = new SparkMaxConfig();

    encoder = motor.getAbsoluteEncoder();

    config.inverted(INTAKE_INVERTED);

    config.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12.0);

    config
        .encoder
        .positionConversionFactor(POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR)
        .uvwMeasurementPeriod(10);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    intakePIDController =
        new ProfiledPIDController(
            IntakeDeployConstants.GAINS.KP(),
            IntakeDeployConstants.GAINS.KI(),
            IntakeDeployConstants.GAINS.KD(),
            CONSTRAINTS);
  }

  @Override
  public void updateInputs(IntakeDeployIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(
        motor,
        encoder::getPosition,
        (position) -> inputs.positionDeg = position > 180 ? position % 360 - 360 : position % 360);
    ifOk(motor, encoder::getVelocity, (velocity) -> inputs.velocityDegPerSec = velocity);

    ifOk(motor, motor::getBusVoltage, (voltage) -> inputs.motorVoltage = voltage);
    ifOk(motor, motor::getMotorTemperature, (temp) -> inputs.motorTemp = temp);
    ifOk(motor, motor::getOutputCurrent, (current) -> inputs.motorCurrent = current);

    inputs.motorConnected = motorDebouncer.calculate(sparkStickyFault);
  }

  @Override
  public void runVoltage(double voltage) {
    motor.setVoltage(Math.max(-12.0, Math.min(12.0, voltage)));
  }

  @Override
  public void stop() {
    runVoltage(0);
  }

  @Override
  public void runPosition(double position, double feedforward) {
    intakeSetpoint = position;
    motor.setVoltage(intakePIDController.calculate(encoder.getPosition(), position) + feedforward);
  }

  @Override
  public void setPID(double KP, double KI, double KD) {
    intakePIDController.setPID(KP, KI, KD);
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
