package frc.robot.subsystems.climb;

import frc.robot.Constants;

public class ClimbConstants {
    public static final Gains GAINS = switch(Constants.currentMode){
        case REAL -> new Gains(0, 0, 0, 0, 0, 0, 0);
        default -> new Gains(0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    };
    
    public static final double TOLERANCE = 2; // degrees
    
    public record Gains(double KP, double KI, double KD, double KS, double KV, double KA, double KG) {}
}
