package frc.robot.subsystems.vision6;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision6.VisionConstants.CameraConstants;

public class VisionIOReal extends VisionIO {
  protected final PhotonCamera camera;
  private final Supplier<Pose2d> robotPoseSupplier;

  private final PhotonPoseEstimator poseEstimator;

  public VisionIOReal(CameraConstants constants, Supplier<Pose2d> robotPoseSupplier) {
    super.constants = constants;
    this.camera = new PhotonCamera(constants.name());

    this.robotPoseSupplier = robotPoseSupplier;
    this.poseEstimator = new PhotonPoseEstimator(FieldConstants.AprilTags.aprilTagFieldLayout,
        constants.robotToCamera());
  }

  @Override
  public void updateInputs() {
    super.connected = camera.isConnected();

    List<PoseObservation> poseObservations = new LinkedList<>();

    // Read new camera results
    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> optVisionEst = poseEstimator.estimateCoprocMultiTagPose(result);

      // If multiple tags are not detected, fallback to single tag estimation
      if (optVisionEst.isEmpty()) {
        optVisionEst = poseEstimator.estimateLowestAmbiguityPose(result);
      }

      // Only add observations that are present
      if (optVisionEst.isPresent()) {
        EstimatedRobotPose visionEst = optVisionEst.get();
        double totalTagDistance = 0;

        // Calculate total distance from each tag to robot
        for (PhotonTrackedTarget target : visionEst.targetsUsed) {
          totalTagDistance += target.getBestCameraToTarget().getTranslation().getNorm();
        }

        // Add pose observation
        poseObservations.add(
          new PoseObservation(
            visionEst.timestampSeconds, // Timestamp
            visionEst.estimatedPose, // 3D pose estimate
            visionEst.targetsUsed.get(0).poseAmbiguity, // Ambiguity 
            visionEst.targetsUsed.size(), // Number of tags
            totalTagDistance / visionEst.targetsUsed.size())); // Average tag distance
      }
    }

    // Save pose observations
    super.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      super.poseObservations[i] = poseObservations.get(i);
    }
  }
}
