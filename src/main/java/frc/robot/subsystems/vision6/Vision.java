package frc.robot.subsystems.vision6;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision6.VisionIO.PoseObservation;

public class Vision {
  private final VisionConsumer consumer;
  private final VisionIOReal[] io;

  private final Alert[] disconnectedAlerts;
  private final Alert cameraDisconnectedAlert = new Alert("One or more cameras are disconnected.", AlertType.kWarning);

  public Vision(VisionConsumer consumer, VisionIOReal... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < io.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  // Functional interface for vision consumer
  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public void updateInputs() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs();
    }

    // Initialize logging values
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {

      cameraDisconnectedAlert.set(!io[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      for (PoseObservation observation : io[cameraIndex].poseObservations) {
        DogLog.log("Vision/" + io[cameraIndex].constants.name() + "/Observation", observation);

        // Check whether to reject pose
        boolean rejectPose = shouldRejectPoseObservation(observation);

        // Log poses
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
          continue;
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Calculate standard deviations
        double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
        double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;

        // Don't trust angular position on single-tag
        if (observation.tagCount() == 1) {
          angularStdDev = 1000.0;
        }

        consumer.accept(observation.pose().toPose2d(), observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera data
      DogLog.log("Vision/" + io[cameraIndex].constants.name() + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      DogLog.log("Vision/" + io[cameraIndex].constants.name() + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      DogLog.log("Vision/" + io[cameraIndex].constants.name() + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }
    // Log summary data
    DogLog.log(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    DogLog.log(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    DogLog.log(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  private boolean shouldRejectPoseObservation(PoseObservation observation) {
    // Should have at least one tag
    if (observation.tagCount() <= 0) {
      DogLog.log("Vision/RejectReason", "No Tags");
      return true;
    }

    // Single tag results can provide more errors than multi-tag
    if (observation.tagCount() == 1) {
      // Single tag results have ambiguity which cause the estimator to pick the wrong
      // location
      if (observation.ambiguity() > VisionConstants.maxAmbiguity) {
        DogLog.log("Vision/RejectReason", "Over Max Ambiguity");
        return true;
      }
      // Single tag results get worse at a distance
      if (observation.averageTagDistance() > VisionConstants.singleTagMaxDistanceMeters) {
        DogLog.log("Vision/RejectReason", "Too Far");
        return true;
      }
    }

    // Result must not be above or below the floor
    if (Math.abs(observation.pose().getZ()) > VisionConstants.maxZError) {
        DogLog.log("Vision/RejectReason", "Above Ground");
      return true;
    }

    // Result must be within field
    return observation.pose().getX() < 0.0
        || observation.pose().getX() > FieldConstants.fieldLengthMeters
        || observation.pose().getY() < 0.0
        || observation.pose().getY() > FieldConstants.fieldWidthMeters;
  }

  // public Matrix<N3, N1>
  // findVisionMeasurementStdDevs(Optional<EstimatedRobotPose> estimatedPose) {
  // if (estimatedPose.isEmpty()) {
  // return VisionConstants.SINGLE_TAG_STD_DEVS;
  // } else {
  // Matrix<N3, N1> estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
  // int numTags = 0;
  // double sumDistance = 0;

  // for (PhotonTrackedTarget target : estimatedPose.get().targetsUsed) {
  // numTags++;
  // Transform3d t3d = target.getBestCameraToTarget();
  // sumDistance += Math.hypot(t3d.getX(), t3d.getY());
  // }

  // if (numTags == 0) {
  // return estStdDevs;
  // } else {
  // double avgDistance = sumDistance / numTags;
  // if (numTags > 1) {
  // estStdDevs = VisionConstants.MULTI_TAG_STD_DEVS;
  // }
  // // else {
  // // estStdDevs = VecBuilder.fill(
  // // VisionConstants.SINGLE_TAG_STD_DEVS.get(0, 0),
  // // VisionConstants.SINGLE_TAG_STD_DEVS.get(1, 0),
  // // Double.MAX_VALUE);
  // // }
  // if (numTags == 1 && avgDistance >
  // VisionConstants.SINGLE_TAG_MAX_DISTANCE_METERS) {
  // estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE,
  // Double.MAX_VALUE);
  // } else {
  // estStdDevs = estStdDevs.times(1 + (avgDistance * avgDistance / 30));
  // }
  // return estStdDevs;
  // }
  // }
  // }
}
