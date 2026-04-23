// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeRollers;

/** Add your docs here. */
public abstract class IntakeRollersIO {
    protected double intakeRollersLeaderVelocity = 0.0;
    protected double intakeRollersLeaderVoltage = 0.0;
    protected double intakeRollersLeaderSupplyCurrent = 0.0;
    protected double intakeRollersLeaderStatorCurrent = 0.0;
    protected double intakeRollersLeaderTemperature = 0.0;

    protected double intakeRollersFollowerVelocity = 0.0;
    protected double intakeRollersFollowerVoltage = 0.0;
    protected double intakeRollersFollowerSupplyCurrent = 0.0;
    protected double intakeRollersFollowerStatorCurrent = 0.0;
    protected double intakeRollersFollowerTemperature = 0.0;


    public void updateInputs(){}

    public void stop() {}
    public void setPosition(double position){}
    public void setVoltage(double voltage){}
    
}
