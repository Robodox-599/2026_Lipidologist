package frc.robot.subsystems.shooter.flywheels;

public class FlywheelsIO {
    protected double position = 0;
    protected double RPS = 0;
    protected double statorCurrent = 0;
    protected double supplyCurrent = 0;
    protected double temperature = 0;
    protected boolean isFlywheelAtSetpoint = false;
    protected double targetRPS = 0;

    public void updateInputs() {}
    public void setRPS(double RPS) {}
    public void setVoltage(double voltage) {}
    public void stop() {}
}
