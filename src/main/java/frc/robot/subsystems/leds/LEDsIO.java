// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

public abstract class LEDsIO {
  protected boolean connected = false;

  public void updateInputs() {}

  public void LEDsStop() {}

  public void LEDsClimbExtend () {}

  public void LEDsClimbRetract () {}

  public void LEDsAutoAlign () {}

  public void LEDsIntakeFuel () {}

  public void LEDsShootFuel () {}

  public void LEDsPulseFuel () {}

}
