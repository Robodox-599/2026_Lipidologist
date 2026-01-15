package frc.robot.subsystems.shooter.flywheels;

public class FlywheelsIO {
    protected double position = 0;
    protected double velocity = 0;
    protected double velocitySetpoint = 0;
    protected boolean isFlywheelAtSetpoint = false;

    public void updateInputs() {}
    public void setRPM(double rpm) {}
    public void setVoltage(double voltage) {}
    public void stop() {}
}
