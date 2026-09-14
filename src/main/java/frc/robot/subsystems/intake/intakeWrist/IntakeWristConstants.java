package frc.robot.subsystems.intake.intakeWrist;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.util.Motor.MotorConstants;

public class IntakeWristConstants {
  public static final String intakeWristCanBus = "rio";
  public static final int intakeWristCANCoderID = 15;

  public static final double rotationalInertia = 0.01;
  public static final double gearRatio = (44.0 / 8.0) * (44.0 / 18.0) * (36.0 / 12.0);
  public static final double absoluteDiscontinuityPoint = 0.7;
  public static final double magnetOffset = 0.16015625;
  public static final double minAngleRotations = 0.0;
  public static final double maxAngleRotations = 0.0;
  public static final double startAngleRad = 0.0;

  public static final double kPSim = 1.0; // 1.28
  public static final double kISim = 0.0;
  public static final double kDSim = 0.0;
  public static final double kVSim = 0.0;
  public static final double kSSim = 0.0;
  public static final double kGSim = 0.0;
  public static final double maxVelocitySim = 100.0;
  public static final double maxAccelerationSim = 16.0;

  public static final double wristLengthMeters = 0.5;
  public static final double wristMassKg = 5.0;
  public static final double agitationTime = 0.4;

  public static final double statorCurrentTrip = 70; // must be changed and tested
  public static final double velocityTrip = 1; // must be changed and tested
  public static final double debounceTime = 0.3; // must be changed and tested

  public static final double totalAgitationTime = 3.0;
  public static final double maxAgitationPosition = 0.25;
  public static final double deltaAgitation = 0.08;

  public static final MotorConstants intakeWristMotor =
      new MotorConstants(
          "IntakeWrist",
          13,
          intakeWristCanBus,
          InvertedValue.Clockwise_Positive,
          NeutralModeValue.Brake,
          gearRatio,
          80.0,
          40.0,
          maxAngleRotations,
          minAngleRotations,
          0.0,
          0.0,
          60.0,
          0.0,
          2.0,
          0.36,
          (Constants.kMotors.kKrakenX60Foc.kV * gearRatio),
          0.39,
          ((12.0 - 0.36 - 0.39) / (Constants.kMotors.kKrakenX60Foc.kV * gearRatio)),
          ((12.0 - 0.36 - 0.39) / (Constants.kMotors.kKrakenX60Foc.kV * gearRatio)),
          FeedbackSensorSourceValue.RemoteCANcoder,
          15,
          gearRatio);
}
