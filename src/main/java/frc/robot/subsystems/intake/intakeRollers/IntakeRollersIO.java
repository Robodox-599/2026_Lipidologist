// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeRollers;

/** Add your docs here. */
public abstract class IntakeRollersIO {
  protected double intakeRollersOneVelocity = 0.0;
  protected double intakeRollersOneVoltage = 0.0;
  protected double intakeRollersOneSupplyCurrent = 0.0;
  protected double intakeRollersOneStatorCurrent = 0.0;
  protected double intakeRollersOneTemperature = 0.0;

  protected double intakeRollersTwoVelocity = 0.0;
  protected double intakeRollersTwoVoltage = 0.0;
  protected double intakeRollersTwoSupplyCurrent = 0.0;
  protected double intakeRollersTwoStatorCurrent = 0.0;
  protected double intakeRollersTwoTemperature = 0.0;

  public void updateInputs() {}

  public void stop() {}

  public void setPosition(double position) {}

  public void setVoltage(double voltage) {}
}
