package frc.robot.subsystems.feeder;

public abstract class FeederIO {
    protected double RPS = 0.0;
    protected double targetRPS = 0.0;
    protected double statorCurrent = 0.0;
    protected double supplyCurrent = 0.0;
    // protected double appliedVolts = 0.0;
    protected double tempCelsius = 0.0;

    public void updateInputs() {}

    public void setFeederVelocity(double RPS) {}

    public void stopFeeder() {}
}