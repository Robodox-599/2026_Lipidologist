package frc.robot.subsystems.indexer;

public abstract class IndexerIO {
    protected double position = 0.0;
    protected double velocity = 0.0;
    protected double statorCurrent = 0.0;
    protected double supplyCurrent = 0.0;
    protected double appliedVolts = 0.0;
    protected double tempCelsius = 0.0;

    public void updateInputs() {}

    public void indexerPulseFuel(double volts) {}

    public void setIndexerVoltage(double volts) {}

    public void stopIndexer() {}
}