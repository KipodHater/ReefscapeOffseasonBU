package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;

import static frc.robot.subsystems.climb.ClimbConstants.*;
import static frc.robot.util.SparkUtil.*;

public class ClimbIOSpark implements ClimbIO {

    private final SparkMax motor;
    private final SparkMaxConfig config;

    private final AbsoluteEncoder climbEncoder;

    private final ProfiledPIDController climbPIDController;

    private final Debouncer motorDebouncer = new Debouncer(0.5);

    private boolean brakeEnabled = true;

    @AutoLogOutput(key = "Climb/Setpoint")
    private Double climbSetpoint;

    public ClimbIOSpark() {
        motor = new SparkMax(MOTOR_ID, SparkMax.MotorType.kBrushless);
        config = new SparkMaxConfig();

        climbEncoder = motor.getAbsoluteEncoder();

        config.inverted(MOTOR_INVERTED);

        config.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12.0);

        config
            .encoder
            .positionConversionFactor(ClimbConstants.POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(ClimbConstants.VELOCITY_CONVERSION_FACTOR)
            .inverted(ENCODER_INVERTED)
            .uvwMeasurementPeriod(10);

        tryUntilOk(motor, 5, () -> motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        climbPIDController = new ProfiledPIDController(
            GAINS.KP(),
            GAINS.KI(),
            GAINS.KD(),
            CLIMB_CONSTRAINTS);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        sparkStickyFault = false;

        ifOk(
            motor,
            climbEncoder::getPosition,
            (position) -> inputs.positionDeg = (position - CLIMB_ENCODER_OFFSET) > 180 ?
            (position - CLIMB_ENCODER_OFFSET) % 360 - 360 : (position - CLIMB_ENCODER_OFFSET) % 360);
        ifOk(motor, climbEncoder::getVelocity, (velocity) -> inputs.velocityDegPerSec = velocity);

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
        motor.stopMotor();
    }

    @Override
    public void runPosition(double position, double feedforward) {
        climbSetpoint = position;
        motor.setVoltage(climbPIDController.calculate(climbEncoder.getPosition(), position) + feedforward);
    }

    @Override
    public void setPID(double KP, double KI, double KD) {
        climbPIDController.setPID(KP, KI, KD);
    }

    @Override
    public void setConstraints(Constraints constraints) {
        climbPIDController.setConstraints(constraints);
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
