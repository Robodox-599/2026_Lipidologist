// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ClimbConstants {
  public static String climbCanbus = "rio";
  public static int climbMotorID = 24;

  public static final double climbGearRatio = 16;

  public static final double climbExtensionPosition = 0;

  public static double kP = 0.0;
  public static double kI = 0.0;
  public static double kD = 0.0;
  public static double kV = Constants.kMotors.kKrakenX60Foc.kV * climbGearRatio;;
  public static double kS = 0.0;
  public static double kG = 0.0;

  public static final double simKP = 0.35;// 0.45
  public static final double simKI = 0;
  public static final double simKD = 0.0;
  public static final double simKS = 0.0;
  public static final double simKV = 0;
  public static final double simKG = 0.0;

  public static double statorCurrent = 20.0;
  public static double supplyCurrent = 70.0;

  public static final double maxVelocity = (12 - kS) / kV;
  public static final double maxAcceleration = maxVelocity;

  public static double tripStatorCurrent = 0.0;

  public static final Transform2d climbTransform = new Transform2d(new Translation2d(Units.inchesToMeters(3.546),
      Units.inchesToMeters(13.568)), new Rotation2d());
}