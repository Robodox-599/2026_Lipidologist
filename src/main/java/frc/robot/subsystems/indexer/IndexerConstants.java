// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

public class IndexerConstants {
  public static final int indexerMotorID = 67;
  public static final String indexerCANBus = "rio";
  public static final double indexerGearRatio = 2;
  public static final double indexerMOI = 0.01; // normal MOI

  public static final double kP = 0.45;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0.03;
  public static final double kV = 0.124 * indexerGearRatio;

  public static final double simKP = 0.45;
  public static final double simKI = 0;
  public static final double simKD = 0;

  public static final double pulseTimeInterval = 0.4;
  public static final double supplyCurrentLimit = 40;
  public static final double maxVelocityRotsPerSec = 100;
  public static final double maxAccelerationRotationsPerSecSQ = maxVelocityRotsPerSec / 2;

  public static final double statorCurrentLimitAmps = 0;
  public static final double supplyCurrentLimitAmps = 0;
}
