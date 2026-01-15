package frc.robot.subsystems.shooter.hood;

public class HoodConstants {
    //motor information
    public static final int hoodMotorID = 0;
    public static final String hoodCANBus = "rio";
    public static final double hoodGearRatio = 0;
   
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

    //current limits 
    public static final double supplyCurrentLimit = 50;
 
    //positon tolerances
    public static final double positionTolerance = 0;
    public static final double hoodMinAngle = 0;
    public static final double hoodMaxAngle = 0;
}
