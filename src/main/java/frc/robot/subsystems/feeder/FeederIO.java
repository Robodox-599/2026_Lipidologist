package frc.robot.subsystems.feeder;

public abstract class FeederIO {
    protected double velocity = 0.0;
    protected double statorCurrent = 0.0;
    protected double supplyCurrent = 0.0;
    protected double appliedVolts = 0.0;
    protected double tempCelsius = 0.0;

    public void updateInputs() {}

    public void setFeederVoltage(double volts) {}

    public void stopFeeder() {}
}