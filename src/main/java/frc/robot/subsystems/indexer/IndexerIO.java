package frc.robot.subsystems.indexer;

public abstract class IndexerIO {
    protected double velocity = 0.0;
    protected double statorCurrent = 0.0;
    protected double supplyCurrent = 0.0;
    protected double voltage = 0.0;
    protected double temperature = 0.0;

    public void updateInputs(){}
    public void stop(){}
    public void setVoltage(double voltage){}
    public void pulse(boolean pulse){}
}