package frc.robot.util;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.Constants.CYCLE_TIME;

public static class accelLimitsLib {

  private static Vector<N2> applyAccLimits(Vector<N2> wantedVelocityRobotOriented, Vector<N2> currentVelocityRobotOriented) {

    Vector<N2> wantedAccRobotOriented = (wantedVelocityRobotOriented.minus(currentVelocityRobotOriented)).div(CYCLE_TIME);    
    // can possibly make this better
    Vector<N2> skidAccel = min(wantedAccRobotOriented.norm(), MAX_SKID_ACCEL) * wantedAccRobotOriented.div(wantedAccRobotOriented.norm());

    // double maxForwardAccel = MAX_ACCELERATION * (currentVelocity. / maxSpeedMetersPerSec);
    double frontAccel = Math.copySign(min(Math.abs(skidAccel.get(0)), MAX_FRONT_ACCEL), skidAccel.get(0));
    double sideAccel = Math.copySign(min(Math.abs(skidAccel.get(1)), MAX_FRONT_ACCEL), skidAccel.get(1));

    Vector<N2> limitedAccRobotOriented = new Vector<N2>(new double[] {frontAccel, sideAccel});

    return currentVelocityRobotOriented.plus(limitedAccRobotOriented.times(CYCLE_TIME));
  }
}