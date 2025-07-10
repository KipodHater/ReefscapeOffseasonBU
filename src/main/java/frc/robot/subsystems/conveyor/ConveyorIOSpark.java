package frc.robot.subsystems.conveyor;

import static frc.robot.subsystems.conveyor.ConveyorConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class ConveyorIOSpark implements ConveyorIO {

  private SparkMax motor;
  private SparkMaxConfig config;

  private RelativeEncoder motorEncoder;

  private Debouncer debouncer = new Debouncer(0.1);
  private Debouncer connectedDebouncer = new Debouncer(0.5);

  private boolean overCurrent;

  private BangBangController controller;

  public ConveyorIOSpark() {
    motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    config = new SparkMaxConfig();

    motorEncoder = motor.getEncoder();

    config
        .idleMode(MOTOR_BRAKE ? IdleMode.kBrake : IdleMode.kCoast)
        .inverted(MOTOR_INVERTED)
        .voltageCompensation(12.0);

    config.encoder.velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    controller = new BangBangController();
    controller.setTolerance(KP);
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(motor, motorEncoder::getVelocity, (value) -> inputs.velocityDegPerSec = value);

    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.motorVoltage = values[0] * values[1]);

    ifOk(motor, motor::getMotorTemperature, (value) -> inputs.motorTemp = value);

    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.motorCurrent = value);

    inputs.motorConnected = connectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void runVoltage(double voltage) {
    motor.setVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  @Override
  public void stop() {
    motor.setVoltage(0);
  }

  @Override
  public void setBangBang(double KP) {
    controller.setTolerance(KP);
  }

  @Override
  public void setBrakeMode(boolean isBrake) {
    config.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void checkCurrent() {

    overCurrent = motor.getOutputCurrent() > currentThreshold;

    runVoltage(debouncer.calculate(overCurrent) ? 0.0 : -4);
  }
}