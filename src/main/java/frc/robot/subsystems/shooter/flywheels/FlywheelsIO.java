package frc.robot.subsystems.shooter.flywheels;

public class FlywheelsIO {
    //right
    protected double positionRight = 0;
    protected double RPSRight = 0;
    protected double statorCurrentRight = 0;
    protected double supplyCurrentRight = 0;
    protected double temperatureRight = 0;
    protected boolean isFlywheelAtSetpointRight = false;
    //center
    protected double positionCenter = 0;
    protected double RPSCenter = 0;
    protected double statorCurrentCenter = 0;
    protected double supplyCurrentCenter = 0;
    protected double temperatureCenter = 0;
    protected boolean isFlywheelAtSetpointCenter = false;
    //left
    protected double positionLeft = 0;
    protected double RPSLeft = 0;
    protected double statorCurrentLeft = 0;
    protected double supplyCurrentLeft = 0;
    protected double temperatureLeft = 0;
    protected boolean isFlywheelAtSetpointLeft = false;

    protected double targetRPS = 0;

    public void updateInputs() {}
    public void setRPS(double RPS) {}
    public void setVoltage(double voltage) {}
    public void stop() {}
}
