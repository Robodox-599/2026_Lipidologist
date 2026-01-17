// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

public class ClimbConstants {
  public static final int climbLeaderMotorID = 67;
  public static final String climbLeaderMotorCANbus = "rio";

  public static final int climbFollowerMotorID = 67;
  public static final String climbFollowerMotorCANbus = "rio";
  
  public static final double climbGearRatio = 2;
  public static final double climbMOI = 2;

  public static final double kP = 0.45;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0.03;
  public static final double kV = 0.124 * climbGearRatio;
  public static final double kG = 0.0;

  public static final double simKP = 0.45;
  public static final double simKI = 0;
  public static final double simKD = 0;

  public static final double supplyCurrentLimit = 40;
  public static final double maxVelocityRotsPerSec = 0;
  public static final double maxAccelerationRotationsPerSecSQ = 0;

  public static final double statorCurrentLimitAmps = 0;
  public static final double supplyCurrentLimitAmps = 0;
  public static final double inchesPerRev = 0.0;
  public static final double climbLowerLimit = 0.0;
  public static final double climbUpperLimit = 100.0;
  public static final double positionToleranceInches = 0.0;

  public static double convertToTicks(double height) {
    return height / inchesPerRev;
  }
}