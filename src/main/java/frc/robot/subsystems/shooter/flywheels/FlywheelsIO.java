package frc.robot.subsystems.shooter.flywheels;

public class FlywheelsIO {
    protected double position = 0;
    protected double statorCurrent = 0;
    protected double RPM = 0;
    protected double targetRPM = 0;
    protected boolean isFlywheelAtSetpoint = false;

    public void updateInputs() {}
    public void setRPM(double RPM) {}
    public void setVoltage(double voltage) {}
    public void stop() {}
}
