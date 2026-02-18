package frc.robot.subsystems.vision6;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision6.VisionConstants.CameraConstants;

public abstract class VisionIO {
    public boolean connected = false;
    public CameraConstants constants = new CameraConstants("", new Transform3d());
    public PoseObservation[] poseObservations = new PoseObservation[0];

    public static record PoseObservation(
        double timestamp,
        Pose3d pose,
        double ambiguity,
        int tagCount,
        double averageTagDistance) 
        {}

    public void updateInputs() {}
}
