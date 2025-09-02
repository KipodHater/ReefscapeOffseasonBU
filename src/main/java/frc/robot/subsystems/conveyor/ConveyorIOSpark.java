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

  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;

  private final SparkMax follower;
  private final SparkMaxConfig followerConfig;

  private Debouncer motorConnectedDebouncer = new Debouncer(0.5);
  private Debouncer followerConnectedDebouncer = new Debouncer(0.5);

  public ConveyorIOSpark() {
    motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    follower = new SparkMax(FOLLOWER_ID, MotorType.kBrushless);
    followerConfig = new SparkMaxConfig();

    motorConfig
        .idleMode(MOTOR_BRAKE ? IdleMode.kBrake : IdleMode.kCoast)
        .inverted(MOTOR_INVERTED)
        .voltageCompensation(12.0)
        .smartCurrentLimit(50);

    followerConfig
        .idleMode(FOLLOWER_BRAKE ? IdleMode.kBrake : IdleMode.kCoast)
        .inverted(FOLLOWER_INVERTED)
        .voltageCompensation(12.0)
        .smartCurrentLimit(50);

    tryUntilOk(
        follower,
        5,
        () ->
            follower.configure(
              followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(motor, motor.getEncoder()::getVelocity, (value) -> inputs.motorVelocityDegPerSec = value);

    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.motorVoltage = values[0] * values[1]);

    ifOk(motor, motor::getMotorTemperature, (value) -> inputs.motorTemp = value);

    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.motorCurrent = value);

    inputs.motorConnected = motorConnectedDebouncer.calculate(!sparkStickyFault);

    sparkStickyFault = false;

    ifOk(follower, follower.getEncoder()::getVelocity, (value) -> inputs.motorVelocityDegPerSec = value);

    ifOk(
        follower,
        new DoubleSupplier[] {follower::getAppliedOutput, follower::getBusVoltage},
        (values) -> inputs.motorVoltage = values[0] * values[1]);

    ifOk(follower, follower::getMotorTemperature, (value) -> inputs.motorTemp = value);

    ifOk(follower, follower::getOutputCurrent, (value) -> inputs.motorCurrent = value);

    inputs.motorConnected = motorConnectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void runVoltage(double voltage) {
    motor.setVoltage(MathUtil.clamp(voltage, -12, 12));
    follower.setVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  @Override
  public void stop() {
    motor.setVoltage(0);
    follower.setVoltage(0);
  }

  @Override
  public void setBrakeMode(boolean isBrake) {
    motorConfig.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
              motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        follower,
        5,
        () ->
            follower.configure(
              followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }
}
