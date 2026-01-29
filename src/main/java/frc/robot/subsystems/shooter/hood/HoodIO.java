package frc.robot.subsystems.shooter.hood;

public class HoodIO {
    protected double position = 0;
    protected double velocity = 0;
    protected double wantedPosition = 0;
    protected double targetPosition = 0;
    protected boolean isHoodInPosition = false;

    public void updateInputs() {}
    public void setPosition(double position) {}
    public void setVoltage(double voltage) {}
    public void stop() {}
}
