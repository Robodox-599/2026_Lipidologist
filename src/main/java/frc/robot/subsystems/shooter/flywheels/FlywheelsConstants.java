package frc.robot.subsystems.shooter.flywheels;

public class FlywheelsConstants {
    //motor information
    public static final int flywheelMotorID = 0;
    public static final String flywheelCANBus = "rio";
    public static final double flywheelGearRatio = 1;
    public static final double flywheelMOI = 0.1;
   
    //real PID
    public static final double flywheelRealkP = 0;
    public static final double flywheelRealkI = 0;
    public static final double flywheelRealkD = 0;
    public static final double flywheelRealkS = 0;
    public static final double flywheelRealkV = 0;
  
    //sim PID
    public static final double flywheelSimkP = 0;
    public static final double flywheelSimkI = 0;
    public static final double flywheelSimkD = 0;
    public static final double flywheelSimkS = 0;
    public static final double flywheelSimkV = 0;

    public static final double flywheelMaxVelocity = 100;
    public static final double flywheelMaxAcceleration = 50;

    //current limits 
    public static final double supplyCurrentLimit = 50;
 
    //velocity tolerance
    public static final double RPMTolerance = 0;

    //setpoints
    public static final double constantRPMSetpoint = 0;
}
