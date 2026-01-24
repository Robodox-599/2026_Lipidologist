package frc.robot.subsystems.vision.camera;

import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.subsystems.vision.camera.Camera.CameraConstants;

public abstract class CameraIO {
  public PhotonPipelineResult result = new PhotonPipelineResult();
  public boolean stale = true;

  public void updateInputs() {}

  public String getName() {
    return "";
  }

  public CameraConstants getCameraConstants() {
    return null;
  }
}
