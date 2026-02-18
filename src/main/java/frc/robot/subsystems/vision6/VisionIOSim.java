package frc.robot.subsystems.vision6;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision6.VisionConstants.CameraConstants;

public class VisionIOSim extends VisionIOReal {
    // Simulation specific
    private static VisionSystemSim visionSim;
    private final PhotonCameraSim cameraSim;
    private final Supplier<Pose2d> robotPoseSupplier;
    
    public VisionIOSim(CameraConstants constants, Supplier<Pose2d> robotPoseSupplier) {
        super(constants, robotPoseSupplier);
        this.robotPoseSupplier = robotPoseSupplier;

        // Initialize vision sim
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(FieldConstants.AprilTags.aprilTagFieldLayout);
        }

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(1600, 1200, Rotation2d.fromDegrees(90));
        cameraProperties.setCalibError(0.4, 0.10);
        cameraProperties.setFPS(25);
        cameraProperties.setAvgLatencyMs(50);
        cameraProperties.setLatencyStdDevMs(15);

        cameraSim = new PhotonCameraSim(super.camera, cameraProperties);

        // Set the following two false if frame time is too long
        cameraSim.enableDrawWireframe(true);
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.setMaxSightRange(Units.feetToMeters(22.0));

        visionSim.addCamera(cameraSim, constants.robotToCamera());
    }

    @Override
    public void updateInputs() {
        visionSim.update(robotPoseSupplier.get());
        super.updateInputs();
    }
}
