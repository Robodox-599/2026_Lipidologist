// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import frc.robot.Constants;

public class FeederConstants {
  public static final int feederMotorID = 17;
  public static final String feederCANBus = "rio";
  public static final double feederGearRatio = 42.0/23.0;
  public static final double feederMOI = 0.01;

  public static final double kP = 0.47;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0.39;
  public static final double kV = 0.117;

  public static final double maxVelocityRotsPerSec = (12 - kS) / kV;
  public static final double maxAccelerationRotationsPerSecSQ = maxVelocityRotsPerSec;

  public static final double supplyCurrentLimitAmps = 40;
  public static final double statorCurrentLimitAmps = 120;

  public static final double stallingStatorCurrentAmps = 20;
  public static final double fuelDebounce = 0.2;
}