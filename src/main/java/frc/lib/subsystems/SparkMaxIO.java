package frc.lib.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static frc.lib.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import frc.lib.util.SparkUtil;
import frc.robot.Robot;

public class SparkMaxIO implements MotorIO {
  protected final SparkMax motor;
  protected final MotorSubsystemConfig config;
  protected final SparkClosedLoopController closedLoopController;

  protected final AbsoluteEncoder absoluteEncoder;
  protected final RelativeEncoder encoder;

  protected final Debouncer motorDebouncer = new Debouncer(0.5);

  protected Angle rotorOffset = Rotations.of(0);

  public SparkMaxIO(MotorSubsystemConfig config) {
    this.config = config;
    this.motor = new SparkMax(config.id, MotorType.kBrushless);

    if (Robot.isSimulation()) {
      this.config.sparkConfig.smartCurrentLimit(200);
      this.config.sparkConfig.openLoopRampRate(0);
      this.config.sparkConfig.closedLoopRampRate(0);
    }

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config.sparkConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    closedLoopController = motor.getClosedLoopController();

    if (config.usingAbsoluteEncoder) absoluteEncoder = motor.getAbsoluteEncoder();
    else absoluteEncoder = null;

    encoder = motor.getEncoder();
    // if(config.usingAbsoluteEncoder) if(absoluteEncoderConnected())
  }

  @Override
  public void updateInputs(MotorInputs inputs) {
    sparkStickyFault = false;

    if (config.usingAbsoluteEncoder)
      if (absoluteEncoderConnected())
        rotorOffset =
            Rotations.of(
                encoder.getPosition()
                    - absoluteEncoder.getPosition() * config.absoluteEncoderToRotorRatio);

    ifOk(
        motor,
        () -> this.unitsToRotor(encoder.getPosition() - rotorOffset.magnitude()),
        (position) -> inputs.unitPosition = position);

    ifOk(
        motor,
        () -> rotorToUnits(encoder.getVelocity()),
        (velocity) -> inputs.velocityUnitsPerSecond = velocity);

    ifOk(
        motor,
        () -> motor.getAppliedOutput() * motor.getBusVoltage(),
        (volts) -> inputs.appliedVolts = volts);

    ifOk(
        motor,
        () -> motor.getOutputCurrent(),
        (statorAmps) -> inputs.currentStatorAmps = statorAmps);

    ifOk(
        motor,
        () -> 0,
        (supplyAmps) ->
            inputs.currentSupplyAmps = supplyAmps); // idk how to get supply current on spark max

    ifOk(
        motor,
        () -> encoder.getPosition(),
        (rawRotorPosition) -> inputs.rawRotorPosition = rawRotorPosition);

    ifOk(motor, () -> motor.getMotorTemperature(), (tempC) -> inputs.temperatureCelsius = tempC);

    motorDebouncer.calculate(sparkStickyFault);
    inputs.motorConnected = motorDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setPositionSetpoint(double units, double ffVolts) {
    closedLoopController.setReference(
        clampPosition(unitsToRotor(units)), ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVolts);
  }

  @Override
  public void setVelocitySetpoint(double unitsPerSecond) {
    closedLoopController.setReference(
        unitsToRotor(unitsPerSecond), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setMaxMotionSetpointPosition(double units, double ffVolts) {
    closedLoopController.setReference(
        clampPosition(unitsToRotor(units)),
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,
        ffVolts);
  }

  @Override
  public void setMaxMotionSetpointVelocity(double unitsPerSecond) {
    closedLoopController.setReference(
        unitsToRotor(unitsPerSecond), ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setVoltageOutput(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setCurrentPositionAsZero() {
    if (!config.usingAbsoluteEncoder)
      encoder.setPosition(0); // could possibly change this for insurance if sensor goes down
  }

  @Override
  public void setCurrentPosition(double positionUnits) {
    encoder.setPosition(unitsToRotor(positionUnits));
  }

  @Override
  public void setNeutralMode(boolean isBrake) {
    if ((motor.configAccessor.getIdleMode() == IdleMode.kBrake) == isBrake) {
      config.sparkConfig.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
      tryUntilOk(
          motor,
          5,
          () ->
              motor.configure(
                  config.sparkConfig,
                  ResetMode.kResetSafeParameters,
                  PersistMode.kPersistParameters));
    }
  }

  @Override
  public void setEnableSoftLimit(boolean fwd, boolean rev) {
    config.sparkConfig.softLimit.forwardSoftLimitEnabled(fwd);
    config.sparkConfig.softLimit.reverseSoftLimitEnabled(rev);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config.sparkConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void setEnableHardLimit(boolean fwd, boolean rev) {
    config.sparkConfig.limitSwitch.forwardLimitSwitchEnabled(fwd);
    config.sparkConfig.limitSwitch.reverseLimitSwitchEnabled(rev);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config.sparkConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  private boolean absoluteEncoderConnected() {
    return true;
  }

  private double unitsToRotor(double units) {
    return units / config.unitToRotorRatio;
  }

  private double rotorToUnits(double rotor) {
    return rotor * config.unitToRotorRatio;
  }

  private double clampPosition(double positionUnits) {
    return Math.min(Math.max(positionUnits, config.kMinPositionUnits), config.kMaxPositionUnits);
  }
}
