package frc.robot.subsystems.feeder;

public abstract class FeederIO {
    protected double velocity = 0.0;
    protected double statorCurrent = 0.0;
    protected double supplyCurrent = 0.0;
    protected double voltage = 0.0;
    protected double temperature = 0.0;


    public void setVelocity(double velocity){};
    public void updateInputs(){};
    public void stop(){};
}