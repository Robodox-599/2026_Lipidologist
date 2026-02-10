package frc.robot.subsystems.shooter.hood;
import com.ctre.phoenix6.CANBus;

public class HoodConstants {
    //motor information
    public static final int hoodMotorID = 0;
    public static final double hoodGearRatio = 1;
    public static final double hoodMOI = 0.1;
   
    //real PID
    public static final double hoodRealkP = 0;
    public static final double hoodRealkI = 0;
    public static final double hoodRealkD = 0;
    public static final double hoodRealkS = 0;
    public static final double hoodRealkV = 0;
    public static final double hoodRealkG = 0;

    //sim PID
    public static final double hoodSimkP = 0;
    public static final double hoodSimkI = 0;
    public static final double hoodSimkD = 0;
    public static final double hoodSimkS = 0;
    public static final double hoodSimkV = 0;
    public static final double hoodSimkG = 0;

    public static final double hoodMaxVelocity = 100;
    public static final double hoodMaxAcceleration = 50;

    //current limits 
    public static final double supplyCurrentLimit = 50;
    public static final double statorCurrentLimit = 50;


    //sim stuff
    public static final double armLengthMeters = 0;
    public static final double startingAngle = 0;
 
    //positon tolerances
    public static final double positionTolerance = 0;
    public static final double hoodMinAngle = 0;
    public static final double hoodMaxAngle = 0;
}
