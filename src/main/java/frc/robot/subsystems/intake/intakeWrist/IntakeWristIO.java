// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeWrist;

/** Add your docs here. */
public class IntakeWristIO {
    protected double targetPosition = 0;
    protected double currentPosition = 0;
    protected double velocity = 0;
    protected double voltage = 0;
    protected double statorCurrent = 0;
    protected double supplyCurrent = 0;
    protected double temperature = 0;

    protected boolean atSetpoint = false;
    protected boolean isWristJammed = false;

    public void updateInputs(){}

    public void stop(){}
    public void setPosition(double position){}
    public double getPosition(){
        return 0.0;
    }
    public void setVoltage(double voltage) {}

}
