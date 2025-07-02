package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;
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

public class PivotIOSpark implements PivotIO {

  private final SparkMax motor;
  private final SparkMaxConfig config;

  private final AbsoluteEncoder pivotEncoder;

  private final ProfiledPIDController pivotPIDController =
      new ProfiledPIDController(GAINS.KP(), GAINS.KI(), GAINS.KD(), PIVOT_CONSTRAINTS);

  private final Debouncer motorDebouncer = new Debouncer(0.5);

  private boolean brakeEnabled = true;

  @AutoLogOutput(key = "Pivot/Setpoint")
  private Double pivotSetpoint;

  public PivotIOSpark() {
    motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);

    config = new SparkMaxConfig();

    pivotEncoder = motor.getAbsoluteEncoder();

    config.inverted(MOTOR_INVERTED);

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
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(
        motor,
        pivotEncoder::getPosition,
        (position) -> inputs.positionDeg = position > 180 ? position % 360 - 360 : position % 360);
    ifOk(motor, pivotEncoder::getVelocity, (velocity) -> inputs.velocityDegPerSec = velocity);

    ifOk(motor, motor::getBusVoltage, (voltage) -> inputs.motorVoltage = voltage);
    ifOk(motor, motor::getMotorTemperature, (temp) -> inputs.motorTemp = temp);
    ifOk(motor, motor::getOutputCurrent, (current) -> inputs.motorCurrent = current);

    inputs.motorConnected = motorDebouncer.calculate(sparkStickyFault);
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
    runVoltage(feedforward + pivotPIDController.calculate(pivotEncoder.getPosition(), position));
  }

  @Override
  public void setPID(double KP, double KI, double KD) {
    pivotPIDController.setPID(KP, KI, KD);
  }

  @Override
  public void setConstraints(Constraints constraints) {
    pivotPIDController.setConstraints(constraints);
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
