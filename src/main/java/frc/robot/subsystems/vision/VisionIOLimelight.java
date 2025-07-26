package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

public class VisionIOLimelight implements VisionIO {

  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;
  private final NetworkTable table;
  private final String name;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  private final IntegerSubscriber validTargetSubscriber;

  private boolean isHighRes = true;
  private boolean isSearchingTags = true;

  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    //         // this.name = name;

    this.rotationSupplier = rotationSupplier;
    isSearchingTags = true;

    table = NetworkTableInstance.getDefault().getTable(name);
    this.name = name;

    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    validTargetSubscriber = table.getIntegerTopic("tv").subscribe(0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});

    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    cropStream(-0.7, 0.7, -0.8, 0.8);
    setHighResPipeline(true);
  }

  //         inputs.latestTargetObservation =
  //             new TargetObservation(Rotation2d.fromDegrees(txSubscriber.get()),
  // Rotation2d.fromDegrees(tySubscriber.get()));

  //     Set<Integer> tagIds = new HashSet<>();
  //     List<PoseObservation> poseObservations = new LinkedList<>();
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault(); // .flush()

    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    Double latestDistance = null;
    for (var rawResult : megatag1Subscriber.readQueue()) {
      for (int i = 11; i < rawResult.value.length; i += 7) {
        tagIds.add((int) rawResult.value[i]);
      }
      if (rawResult.value.length == 0) {
        continue;
      }
      poseObservations.add(
          new PoseObservation(
              rawResult.timestamp * (1.0e-6) - rawResult.value[6] * (1.0e-3),
              parsePose(rawResult.value),
              rawResult.value.length >= 18 ? rawResult.value[17] : 0.0,
              (int) rawResult.value[7],
              rawResult.value[9],
              rawResult.value[9],
              0.1));
      latestDistance = rawResult.value[9];
    }

    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
    if (validTargetSubscriber.get() == 1) {
      cropStream(
          txSubscriber.get() - 0.5,
          txSubscriber.get() + 0.5,
          tySubscriber.get() - 0.5,
          tySubscriber.get() + 0.5);
      if (latestDistance != null) {
        if (latestDistance < 3.0) setHighResPipeline(false);
        else if (latestDistance > 3.5) setHighResPipeline(true);
      }
      isSearchingTags = false;
    } else if (!isSearchingTags) {
      cropStream(-0.7, 0.7, -0.8, 0.8);
      setHighResPipeline(true);
      isSearchingTags = true;
    }
  }

  private void setHighResPipeline(boolean highRes) {
    if (highRes == isHighRes) return;
    isHighRes = highRes;
    table
        .getEntry("pipeline")
        .setNumber(highRes ? VisionConstants.highResPipeline : VisionConstants.lowResPipeline);
    System.out.println("highRes");
  }

  private void cropStream(
      double minX, double maxX, double minY, double maxY) { // all values are (-1.0, 1.0)
    double[] cropArray = {minX, maxX, minY, maxY};
    table.getEntry("crop").setDoubleArray(cropArray);
    System.out.println("got here!!!");
  }

  public String getName() {
    return name;
  }

  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
