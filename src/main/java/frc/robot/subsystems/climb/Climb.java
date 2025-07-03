package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import static frc.robot.subsystems.climb.ClimbConstants.*;
public class Climb {

    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private final ArmFeedforward climbFeedforwardController =
        new ArmFeedforward(ClimbConstants.GAINS.KS(), ClimbConstants.GAINS.KG(), ClimbConstants.GAINS.KV());

    private enum ClimbStates {
        IDLE(null),
        OPEN(100.0),
        CLIMBING(200.0),
        HOLD_POSITION(null);

        Double position;
        ClimbStates(Double position) {
            this.position = position;
        }
        public Double position() {
            return position;
        }
    }

    @AutoLogOutput(key = "Climb/currentState")
    private ClimbStates currentState = ClimbStates.IDLE;

    public Climb(ClimbIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);

        stateMachine();
    }

    public void stateMachine(){
        switch(currentState){
            case IDLE -> io.stop();
            
            case OPEN -> io.runPosition(ClimbStates.OPEN.position(), 0);

            case CLIMBING -> io.runPosition(ClimbStates.CLIMBING.position(), 0);

            case HOLD_POSITION -> io.runVoltage(1);
        }
    }

    public boolean isReadyToClimb() {
        return Math.abs(inputs.positionDeg - ClimbStates.OPEN.position()) < TOLERANCE;
    }
}
