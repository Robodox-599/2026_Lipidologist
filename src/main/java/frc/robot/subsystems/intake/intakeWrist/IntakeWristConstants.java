// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeWrist;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class IntakeWristConstants {
    public static final int intakeWristMotorID = 0;
    public static final int intakeWristCanCoderID = 0;
    public static final String intakeWristCanBus = "rio";

    public static final double rotationalInertia = 0.01;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kV = 0;
    public static final double kS = 0;
    public static final double kG = 0;

    public static final double maxVelocity = 0.0;
    public static final double maxAcceleration = 0.0;
    public static final double absoluteDiscontinuityPoint = 0.0;
    public static final double magnetOffset = 0.0;
    public static final double gearRatio = 5;
    public static final double supplyCurrentLimit = 0.0;
    public static final double statorCurrentLimit = 0.0;
    public static final double minAngleRad = 0;
    public static final double maxAngleRad = Math.PI;
    public static final double startAngleRad = 0;

    public static final double kPSim = 1; //1.28
    public static final double kISim = 0;
    public static final double kDSim = 0;

    public static final double kVSim = 0.0;
    public static final double kSSim = 0.0;
    public static final double kGSim = 0.0;
    public static final double maxVelocitySim = 100;
    public static final double maxAccelerationSim = 16;

    public static final double wristLengthMeters = 0.5;
    public static final double wristMassKg = 5;


}
