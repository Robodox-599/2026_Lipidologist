package frc.robot.subsystems.intake.intakeRollers;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.Motor.MotorConstants;

public class IntakeRollersConstants {
  public static final String intakeRollersCanBus = "rio";

  public static final double rotationalInertia = 0.01;

  public static final MotorConstants intakeRollersLeader = new MotorConstants(
      "IntakeRollersLeader",
      14, 
      intakeRollersCanBus,
      InvertedValue.CounterClockwise_Positive,
      NeutralModeValue.Brake,
      1.0, 
      50.0, 20.0, 
      null, null, 
      0.0, 0.0, 
      0.0, 0.0, 0.0, 
      0.0, 0.0, 0.0, 
      0.0, 
      0.0, 
      null, 0, null
  );

  public static final MotorConstants intakeRollersFollower = new MotorConstants(
      "IntakeRollersFollower",
      25, 
      intakeRollersCanBus,
      InvertedValue.CounterClockwise_Positive,
      NeutralModeValue.Brake,
      1.0, 
      50.0, 20.0, 
      0.0, 0.0, 
      0.0, 0.0, 
      0.0, 0.0, 0.0, 
      0.0, 0.0, 0.0, 
      0.0, 
      0.0, 
      FeedbackSensorSourceValue.FusedCANcoder, 0, null
  );
}

