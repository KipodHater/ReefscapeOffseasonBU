package frc.robot.subsystems.intakeRollers;

import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.intakeRollers.IntakeRollersConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;

public class IntakeRollersIOSpark implements IntakeRollersIO {
    private final SparkMax motor;
    private SparkMaxConfig config;
    private final RelativeEncoder motorEncoder;
    // private final SparkClosedLoopController motorController;

    private SimpleMotorFeedforward ffController;

    private final Debouncer sparkConnectedDebounce = new Debouncer(0.5);

    public IntakeRollersIOSpark() {
        motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
        motor.clearFaults();

        motorEncoder = motor.getEncoder();

        config = new SparkMaxConfig();
        config.inverted(MOTOR_INVERTED)
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12.0);

        config.encoder.velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

        tryUntilOk(
            motor,
            5,
            () ->
                motor.configure(
                    config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(IntakeRollersIOInputs inputs) {
        sparkStickyFault = false;

    ifOk(motor, motorEncoder::getVelocity, (value) -> inputs.velocityDegPerSec = value);

    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.motorVoltage = values[0] * values[1]);

    ifOk(motor, motor::getMotorTemperature, (value) -> inputs.motorTemp = value);

    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.motorCurrent = value);

    inputs.motorConnected = sparkConnectedDebounce.calculate(!sparkStickyFault);
    }

    @Override
    public void runVoltage(double voltage) {
        motor.setVoltage(Math.min(-12.0, Math.max(12.0, voltage)));
    }

    @Override
    public void stop() {
        motor.setVoltage(0);
    }
}
