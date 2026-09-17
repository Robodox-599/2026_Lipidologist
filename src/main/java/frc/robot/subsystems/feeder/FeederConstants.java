package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.Motor.MotorConstants;

public class FeederConstants {
  public static final String feederCANBus = "rio";
  public static final double feederMOI = 0.01;

  public static final double stallingStatorCurrentAmps = 90;
  public static final double jammedRPSTolerance = 4;
  public static final double fuelDebounce = 0.5;

  public static final MotorConstants feederMotor = new MotorConstants(
      "Feeder",
      17, 
      feederCANBus,
      InvertedValue.CounterClockwise_Positive,
      NeutralModeValue.Brake,
      42.0 / 23.0,
      120.0, 40.0, 
      null, null,
      0.0, 
      0.0,
      11.0, 0.0, 0.0, 
      6.5, 0.031, 0.0,
      0.0, 
      0.0, 
      null, null, null
  );
}

