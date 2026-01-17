// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeWrist;

import frc.robot.subsystems.intake.intakeWrist.IntakeWrist.WantedState;

/** Add your docs here. */
public class IntakeWristIO {
    protected double position = 0;
    protected double velocity = 0;
    protected double voltage = 0;
    protected double statorCurrent = 0;
    protected double supplyCurrent = 0;
    protected double Temperature = 0;
    public double temperature;

    public double wantedPosition = 0;

    public boolean isWristInPosition = false;
    public WantedState wantedState;

    public void updateInputs(){}

    public void stop(){}
    public void setPosition(double position){}
    public void setVelocity(){}

}
