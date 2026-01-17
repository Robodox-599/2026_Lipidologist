// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeRollers;

/** Add your docs here. */
public abstract class IntakeRollersIO {
    protected double position = 0.0;
    protected double velocity = 0.0;
    protected double voltage = 0.0;
    protected double supplyCurrent = 0.0;
    protected double statorCurrent = 0.0;
    protected double temperature = 0.0;


    public void updateInputs(){}

    public void stop() {}
    public void setPosition(double position){}
    public void setVelocity(double velocity){}
    
}
