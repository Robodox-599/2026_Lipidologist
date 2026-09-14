package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.Motor.MotorConstants;

public class IndexerConstants {
  public static final String indexerCANBus = "rio";
  public static final double indexerMOI = 0.01; // normal MOI

  public static final double simKP = 0.45;
  public static final double simKI = 0;
  public static final double simKD = 0;

  public static final double pulseTimeInterval = 0.4;
  public static final double maxVelocityRotsPerSec = 100;
  public static final double maxAccelerationRotationsPerSecSQ = maxVelocityRotsPerSec / 2;
  public static final double stallingStatorCurrentAmps = 20;
  public static final double jammedVelocityTolerance = 2;
  public static final double fuelDebounce = 0.2;

  public static final MotorConstants indexerMotor =
      new MotorConstants(
          "Indexer",
          16,
          indexerCANBus,
          InvertedValue.CounterClockwise_Positive,
          NeutralModeValue.Brake,
          2.0,
          120.0,
          25.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.45,
          0.0,
          0.0,
          0.03,
          (0.124 * 2.0),
          0.0,
          maxVelocityRotsPerSec,
          maxAccelerationRotationsPerSecSQ,
          FeedbackSensorSourceValue.FusedCANcoder,
          0,
          null);
}
