package frc.robot.subsystems.shooter.flywheels;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.Motor.MotorConstants;

public class FlywheelsConstants {
  public static final String flywheelCANBus = "rio";

  public static final double flywheelMOI = 0.1;
  public static final double flywheelMaxVelocity = 100;
  public static final double flywheelMaxAcceleration = 50;
  public static final double RPSTolerance = 3.0;
  public static final double idleRPS = 0;

  public static final MotorConstants flywheelLeader =
      new MotorConstants(
          "FlywheelLeader",
          29,
          flywheelCANBus,
          InvertedValue.Clockwise_Positive,
          NeutralModeValue.Coast,
          1.0 / 1.2,
          40.0,
          40.0,
          null,
          null,
          0.0,
          0.0,
          9.0,
          0.0,
          0.0,
          7.3,
          0.059,
          0.0);

  public static final MotorConstants flywheelBottomLeft =
      new MotorConstants(
          "FlywheelBottomLeft",
          26,
          flywheelCANBus,
          InvertedValue.Clockwise_Positive,
          NeutralModeValue.Coast,
          1.0 / 1.2,
          40.0,
          40.0,
          null,
          null,
          0.0,
          0.0,
          9.0,
          0.0,
          0.0,
          7.3,
          0.059,
          0.0);

  public static final MotorConstants flywheelTopRight =
      new MotorConstants(
          "FlywheelTopRight",
          27,
          flywheelCANBus,
          InvertedValue.Clockwise_Positive,
          NeutralModeValue.Coast,
          1.0 / 1.2,
          40.0,
          40.0,
          null,
          null,
          0.0,
          0.0,
          9.0,
          0.0,
          0.0,
          7.3,
          0.059,
          0.0);

  public static final MotorConstants flywheelBottomRight =
      new MotorConstants(
          "FlywheelBottomRight",
          28,
          flywheelCANBus,
          InvertedValue.Clockwise_Positive,
          NeutralModeValue.Coast,
          1.0 / 1.2,
          40.0,
          40.0,
          null,
          null,
          0.0,
          0.0,
          9.0,
          0.0,
          0.0,
          7.3,
          0.059,
          0.0);
}
