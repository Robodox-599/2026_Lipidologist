package frc.robot.subsystems.feeder;

import edu.wpi.first.math.filter.Debouncer;

public abstract class FeederIO {
    protected double velocity = 0.0;
    protected double statorCurrent = 0.0;
    protected double supplyCurrent = 0.0;
    protected double voltage = 0.0;
    protected double temperature = 0.0;

    protected boolean isFeederJammed = false;

    public void setVelocity(double velocity){};
    public void updateInputs(){};
    public void stop(){};
}