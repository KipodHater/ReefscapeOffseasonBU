package frc.lib;

import static frc.robot.Constants.CYCLE_TIME;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;

public class accelLimitsLib {

  public static ChassisSpeeds applyAccLimits(
      ChassisSpeeds wantedChassisVelocityRobotOriented,
      ChassisSpeeds currentChassisVelocityRobotOriented) {

    Vector<N2> wantedVelocityRobotOriented =
        VecBuilder.fill(
            wantedChassisVelocityRobotOriented.vxMetersPerSecond,
            wantedChassisVelocityRobotOriented.vyMetersPerSecond);
    Vector<N2> currentVelocityRobotOriented =
        VecBuilder.fill(
            currentChassisVelocityRobotOriented.vxMetersPerSecond,
            currentChassisVelocityRobotOriented.vyMetersPerSecond);
    Vector<N2> wantedAccRobotOriented =
        (wantedVelocityRobotOriented.minus(currentVelocityRobotOriented)).div(CYCLE_TIME);
    // can possibly make this better
    Vector<N2> skidAccel =
        wantedAccRobotOriented
            .div(wantedAccRobotOriented.norm())
            .times(Math.min(wantedAccRobotOriented.norm(), MAX_SKID_ACCEL));

    double currentVelocityForward = currentVelocityRobotOriented.norm();
    double wantedVelocityForward =
        (currentVelocityRobotOriented.dot(wantedVelocityRobotOriented)) / (currentVelocityForward);
    Vector<N2> notForwardWantedVelocity =
        wantedVelocityRobotOriented.minus(
            currentVelocityRobotOriented.div(currentVelocityForward).times(wantedVelocityForward));

    double maxForwardAccel = MAX_ACCELERATION * (1 - currentVelocityForward / maxSpeedMetersPerSec);
    double maxVelocityForward = currentVelocityForward + maxForwardAccel * CYCLE_TIME;
    double minVelocityForward = currentVelocityForward - maxForwardAccel * CYCLE_TIME;
    wantedVelocityForward =
        Math.max(minVelocityForward, Math.min(maxVelocityForward, wantedVelocityForward));

    Vector<N2> projectionVector =
        currentVelocityRobotOriented
            .div(currentVelocityRobotOriented.norm())
            .times(wantedVelocityForward);

    wantedVelocityRobotOriented = projectionVector.plus(notForwardWantedVelocity);

    double robotFrontAccel =
        Math.copySign(Math.min(Math.abs(skidAccel.get(0)), MAX_FRONT_ACCEL), skidAccel.get(0));
    double robotSideAccel =
        Math.copySign(Math.min(Math.abs(skidAccel.get(1)), MAX_FRONT_ACCEL), skidAccel.get(1));

    Vector<N2> limitedAccRobotOriented = VecBuilder.fill(robotFrontAccel, robotSideAccel);

    wantedVelocityRobotOriented =
        currentVelocityRobotOriented.plus(limitedAccRobotOriented.times(CYCLE_TIME));

    return new ChassisSpeeds(
        wantedVelocityRobotOriented.get(0),
        wantedVelocityRobotOriented.get(1),
        wantedChassisVelocityRobotOriented.omegaRadiansPerSecond);
  }
}
