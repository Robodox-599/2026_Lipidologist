// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import dev.doglog.DogLog;

public class LEDsIOSim extends LEDsIO {
  /** Creates a new LEDIOSim. */
  public LEDsIOSim() {}

  @Override
  public void updateInputs() {
    super.connected = true;

    DogLog.log("LEDs/Connected", super.connected);
  }
}
