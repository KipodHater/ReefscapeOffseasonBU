package frc.lib;

import static frc.robot.Constants.CYCLE_TIME;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

public class accelLimitsLib {

  private static Vector<N2> applyAccLimits(
      Vector<N2> wantedVelocityRobotOriented, Vector<N2> currentVelocityRobotOriented) {

    Vector<N2> wantedAccRobotOriented =
        (wantedVelocityRobotOriented.minus(currentVelocityRobotOriented)).div(CYCLE_TIME);
    // can possibly make this better
    Vector<N2> skidAccel =
        wantedAccRobotOriented
            .div(wantedAccRobotOriented.norm())
            .times(Math.min(wantedAccRobotOriented.norm(), MAX_SKID_ACCEL));

    // double maxForwardAccel = MAX_ACCELERATION * (currentVelocity. / maxSpeedMetersPerSec);
    double frontAccel =
        Math.copySign(Math.min(Math.abs(skidAccel.get(0)), MAX_FRONT_ACCEL), skidAccel.get(0));
    double sideAccel =
        Math.copySign(Math.min(Math.abs(skidAccel.get(1)), MAX_FRONT_ACCEL), skidAccel.get(1));

    Vector<N2> limitedAccRobotOriented = VecBuilder.fill(frontAccel, sideAccel);

    return currentVelocityRobotOriented.plus(limitedAccRobotOriented.times(CYCLE_TIME));
  }
}
