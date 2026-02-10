// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

public class FeederConstants {
  public static final int feederMotorID = 41;
  public static final String feederCANBus = "rio";
  public static final double feederGearRatio = 2;
  public static final double feederMOI = 0.01;

  public static final double kP = 0.45;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0.03;
  public static final double kV = 0.124 * feederGearRatio;

  public static final double supplyCurrentLimit = 40;
}