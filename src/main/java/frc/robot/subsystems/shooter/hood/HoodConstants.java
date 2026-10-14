package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.util.Motor.MotorConstants;

public class HoodConstants {
  public static final String hoodCANBus = "rio";

  public static final double hoodMOI = 0.1;
  public static final double stowPosition = 0.01;
  public static final double armLengthMeters = 0;
  public static final double startingAngleRotations = 0;
  public static final double positionTolerance = 0.015;
  public static final double hoodMagnetOffset = -0.974609375;
  public static final double absoluteDiscontinuityPoint = 0.7;
  public static final double hoodMinAngleRotations = 0.0;
  public static final double hoodMaxAngleRotations = 0.12;
  public static final int hoodCANCoderID = 19;

  public static final double hoodSimkP = 7;
  public static final double hoodSimkI = 0;
  public static final double hoodSimkD = 6;
  public static final double hoodSimkS = 0;
  public static final double hoodSimkV = 0;
  public static final double hoodSimkG = 0;

  public static final MotorConstants hoodMotor =
      new MotorConstants(
          "Hood",
          18,
          hoodCANBus,
          InvertedValue.Clockwise_Positive,
          NeutralModeValue.Brake,
          51.0,
          120.0,
          25.0,
          hoodMaxAngleRotations,
          hoodMinAngleRotations,
          0.0,
          0.0,
          250.0,
          0.0,
          0.0,
          0.31,
          51.0 * Constants.kMotors.kKrakenX60Foc.kV,
          0.18,
          ((12.0 - 0.31) / (51.0 * Constants.kMotors.kKrakenX60Foc.kV)),
          (((12.0 - 0.31) / (51.0 * Constants.kMotors.kKrakenX60Foc.kV)) * 2.0),
          FeedbackSensorSourceValue.RemoteCANcoder,
          hoodCANCoderID,
          null);
}
