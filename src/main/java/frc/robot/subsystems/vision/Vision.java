// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.camera.Camera;

public class Vision {
  private final Camera[] cameras;

  public Vision(Camera... cameras) {
    this.cameras = cameras;
  }

  public void updateInputs() {
    for (Camera camera : cameras) {
      camera.updateInputs();
    }
  }
}
