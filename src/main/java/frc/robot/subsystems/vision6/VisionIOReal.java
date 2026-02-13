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
  private final PhotonCamera camera;

  // private final Supplier<Pose2d> robotPoseSupplier;
  private final PhotonPoseEstimator poseEstimator;

  public VisionIOReal(CameraConstants constants) {
    super.constants = constants;
    this.camera = new PhotonCamera(constants.name());

    this.poseEstimator = new PhotonPoseEstimator(FieldConstants.AprilTags.aprilTagFieldLayout,
        constants.robotToCamera());
  }

  // public VisionIOReal(CameraConstants constants, Supplier<Pose2d> robotPoseSupplier) {
  //   super.constants = constants;
  //   this.camera = new PhotonCamera(constants.name());

  //   this.robotPoseSupplier = robotPoseSupplier;
  //   this.poseEstimator = new PhotonPoseEstimator(FieldConstants.AprilTags.aprilTagFieldLayout,
  //       constants.robotToCamera());
  // }

  @Override
  public void updateInputs() {
    super.connected = camera.isConnected();

    List<PoseObservation> poseObservations = new LinkedList<>();

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> optVisionEst = poseEstimator.estimateCoprocMultiTagPose(result);

      if (optVisionEst.isEmpty()) {
        optVisionEst = poseEstimator.estimateLowestAmbiguityPose(result);
      }

      if (optVisionEst.isPresent()) {
        EstimatedRobotPose visionEst = optVisionEst.get();
        double totalTagDistance = 0;

        for (PhotonTrackedTarget target : visionEst.targetsUsed) {
          totalTagDistance = target.getBestCameraToTarget().getTranslation().getNorm();
        }

        poseObservations.add(
          new PoseObservation(
            visionEst.timestampSeconds, 
            visionEst.estimatedPose,
            visionEst.targetsUsed.get(0).poseAmbiguity, 
            visionEst.targetsUsed.size(),
            totalTagDistance / visionEst.targetsUsed.size()));
      }
    }

    super.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      super.poseObservations[i] = poseObservations.get(i);
    }
  }
}
