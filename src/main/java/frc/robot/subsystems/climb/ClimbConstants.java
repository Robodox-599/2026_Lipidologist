// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import frc.robot.Constants;

public class ClimbConstants {
  public static final int climbMotorID = 24;
  public static final String climbMotorCANbus = "rio";
  
  public static final double climbGearRatio = 16;
  public static final double climbMOI = 0.01;

  public static final double kP = 0.0;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0.0;
  public static final double kV = Constants.kMotors.kKrakenX60Foc.kV * climbGearRatio;
  public static final double kG = 0.0;

  public static final double simKP = 0.35;//0.45
  public static final double simKI = 0;
  public static final double simKD = 0.0;
  public static final double simKS = 0.0;
  public static final double simKV = 0;
  public static final double simKG = 0.0;

  public static final double maxVelocityRotsPerSec = (12 - kS) / kV;
  public static final double maxAccelerationRotationsPerSecSQ = maxVelocityRotsPerSec;

  public static final double statorCurrentLimitAmps = 20;
  public static final double supplyCurrentLimitAmps = 70;
  public static final double inchesPerRev = 1.0;
  public static final double climbLowerLimit = 0.0;
  public static final double climbUpperLimit = 100.0;
  public static final double positionToleranceInches = 0.5;

  public static double convertToTicks(double height) {
    return height / inchesPerRev;
  }
}