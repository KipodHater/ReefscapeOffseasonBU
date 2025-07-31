package frc.robot.subsystems.leds;

import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class leds extends SubsystemBase {
    public enum ledsState {
        OFF(LEDPattern.kOff),
        ALGAE(LEDPattern.solid(Color.kGhostWhite)),
        CORAL(LEDPattern.solid(Color.kAqua)),
        RED(LEDPattern.solid(Color.kRed)),
        PURPLE(LEDPattern.solid(Color.kPurple)),
        BLINK_PURPLE(LEDPattern.solid(Color.kPurple).blink(Time.ofBaseUnits(.2, Units.Seconds), Time.ofBaseUnits(.1,  Units.Seconds))),
        BLUE(LEDPattern.solid(Color.kBlue));


        LEDPattern value;

        ledsState(LEDPattern value) {
            this.value = value;
        }

        public LEDPattern pattern() {
            return this.value;
        }
    }

    private final AddressableLED addressableLEDs = new AddressableLED(0);

    public leds() {

    }
}
