package frc.robot.subsystems.shooter.hood;

public class HoodConstants {
  // motor information
  public static final int hoodMotorID = 18;
  public static final int hoodCANCoderID = 19;
  public static final double hoodGearRatio = 51;
  public static final double hoodMOI = 0.1;

  // positions
  public static final double stowPosition = 0.0;

  // real PID
  public static final double hoodRealkP = 90;
  public static final double hoodRealkI = 0;
  public static final double hoodRealkD = 5;
  public static final double hoodRealkS = 0.5;
  public static final double hoodRealkV = 6.2;
  public static final double hoodRealkG = 0.2;

  // sim PID
  public static final double hoodSimkP = 7;
  public static final double hoodSimkI = 0;
  public static final double hoodSimkD = 6;
  public static final double hoodSimkS = 0;
  public static final double hoodSimkV = 0;
  public static final double hoodSimkG = 0;

  public static final double hoodMaxVelocity = (12 - hoodRealkS) / hoodRealkV;
  public static final double hoodMaxAcceleration = hoodMaxVelocity * 2;

  // current limits
  public static final double supplyCurrentLimit = 20;
  public static final double statorCurrentLimit = 50; // reduce if overshooting too much

  // sim stuff
  public static final double armLengthMeters = 0;
  public static final double startingAngleRotations = 0;

  // positon tolerances
  public static final double positionTolerance = 0.015;
  public static final double hoodMinAngleRotations = 0;
  public static final double hoodMaxAngleRotations = 0.12;

  // cancoderstuff
  public static final double hoodMagnetOffset = -0.9735;
  public static final double absoluteDiscontinuityPoint = 0.98;
}
