// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeRollers; // make sure the folder name is all lowercase

/** Add your docs here. */
public class IntakeRollersConstants {
    public static final int intakeRollersMotorID = 14;
    public static final String intakeRollersCanBus = "rio";

    public static final int intakeRollersGearRatio = 1;

    public static final double rotationalInertia = 0.01;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;

    public static double supplyCurrentLimit = 70;
    public static double statorCurrentLimit = 200;
}
