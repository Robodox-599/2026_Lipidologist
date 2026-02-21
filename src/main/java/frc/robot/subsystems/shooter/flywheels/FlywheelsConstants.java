package frc.robot.subsystems.shooter.flywheels;

public class FlywheelsConstants {
        // motor information
        public static final double flywheelGearRatio = 1/1.2;
        public static final double flywheelMOI = 0.1;
        public static final String flywheelCANBus = "rio";
       
        public static final int flywheelMotorIDRight = 0;
        public static final int flywheelMotorIDCenter = 0;
        public static final int flywheelMotorIDLeft = 0;


        // real PID
        //right
        public static final double flywheelRightkP = 0.6;
        public static final double flywheelRightkI = 0;
        public static final double flywheelRightkD = 0;
        public static final double flywheelRightkS = 0.45;
        public static final double flywheelRightkV = 0.119;

        //center
        public static final double flywheelCenterkP = 0.5;
        public static final double flywheelCenterkI = 0;
        public static final double flywheelCenterkD = 0;
        public static final double flywheelCenterkS = 0.27;
        public static final double flywheelCenterkV = 0.12;

        //Left
        public static final double flywheelLeftkP = 0.65;
        public static final double flywheelLeftkI = 0;
        public static final double flywheelLeftkD = 0;
        public static final double flywheelLeftkS = 0.26;
        public static final double flywheelLeftkV = 0.119;

        // // sim PID
        // public static final double flywheelSimkP = 1.25;
        // public static final double flywheelSimkI = 0;
        // public static final double flywheelSimkD = 0;
        // public static final double flywheelSimkS = 0;
        // public static final double flywheelSimkV = 0;

        public static final double flywheelMaxVelocity = 100;
        public static final double flywheelMaxAcceleration = 50;

        // current limits
        public static final double supplyCurrentLimit = 50;
        public static final double statorCurrentLimit = 50;

        // velocity tolerance
        public static final double RPSTolerance = 0.01;

        // setpoints
        public static final double idleRPS = 0;

        // public record FlywheelConstants(String name, int motorID, String CANBus, double kP, double kI, double kD,
        //                 double kS,
        //                 double kV, InvertedValue invert) {
        // }

        // public static final FlywheelConstants LeftFlywheel = new FlywheelConstants("LeftFlywheel", 20, "rio", 0.65, 0, 0,
        //                 .26, 0.119,
        //                 InvertedValue.Clockwise_Positive);
        // public static final FlywheelConstants MiddleFlywheel = new FlywheelConstants("MiddleFlywheel", 21, "rio", 0.5, 0,
        //                 0, 0.27,
        //                 0.12, InvertedValue.CounterClockwise_Positive);
        // public static final FlywheelConstants RightFlywheel = new FlywheelConstants("RightFlywheel", 22, "rio", 0.6, 0, 0,
        //                 0.45,
        //                 0.119, InvertedValue.CounterClockwise_Positive);

        // public static final FlywheelConstants LeftFlywheelSim = new FlywheelConstants("LeftFlywheel", 20, "rio", 1.25,
        //                 0, 0, 0, 0,
        //                 InvertedValue.Clockwise_Positive);
        // public static final FlywheelConstants MiddleFlywheelSim = new FlywheelConstants("MiddleFlywheel", 21, "rio",
        //                 1.25, 0, 0, 0,
        //                 0, InvertedValue.CounterClockwise_Positive);
        // public static final FlywheelConstants RightFlywheelSim = new FlywheelConstants("RightFlywheel", 22, "rio", 1.25,
        //                 0, 0, 0,
        //                 0, InvertedValue.CounterClockwise_Positive);
}
