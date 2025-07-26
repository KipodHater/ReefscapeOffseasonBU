package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.gripper.GripperConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class GripperIOSpark implements GripperIO {
  private final SparkMax motor;
  private SparkMaxConfig config;
  private final RelativeEncoder motorEncoder;
  private final SparkClosedLoopController motorController;

  private SimpleMotorFeedforward ffController;

  private final Debouncer sparkConnectedDebounce = new Debouncer(0.5);

  public GripperIOSpark() {
    motor = new SparkMax(K_SPARK_ID, MotorType.kBrushless);
    motor.clearFaults();

    motorEncoder = motor.getEncoder();
    motorController = motor.getClosedLoopController();

    config = new SparkMaxConfig();
    config.inverted(K_INVERTED);
    config
        .idleMode(K_BRAKE ? IdleMode.kBrake : IdleMode.kCoast)
        .voltageCompensation(12.0); // .smartCurrentLimit(...)

    config.encoder.velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(GAINS.KP(), GAINS.KI(), GAINS.KD(), 0);

    ffController = new SimpleMotorFeedforward(GAINS.KS(), GAINS.KV(), GAINS.KA());

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(GripperIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(motor, motor.getEncoder()::getVelocity, (value) -> inputs.gripperMotorRPM = value);

    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.gripperMotorVoltage = values[0] * values[1]);

    ifOk(motor, motor::getMotorTemperature, (value) -> inputs.gripperMotorTemp = value);

    inputs.motorConnected = sparkConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setSpeedRPM(double speedRPM) {
    double ffVoltage = ffController.calculate(motorEncoder.getVelocity());
    motorController.setReference(
        speedRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVoltage, ArbFFUnits.kVoltage);
  }

  @Override
  public void setVoltageOpenLoop(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void stop() {
    motor.setVoltage(0);
  }

  @Override
  public void setPID(double kp, double ki, double kd) {
    config.closedLoop.pid(kp, ki, kd);
  }

  @Override
  public void setFF(double ks, double kv, double ka) {
    ffController = new SimpleMotorFeedforward(ks, kv, ka);
  }
}
