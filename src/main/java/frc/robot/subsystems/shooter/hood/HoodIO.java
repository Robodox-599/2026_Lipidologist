package frc.robot.subsystems.shooter.hood;

public class HoodIO {
    protected double position = 0;
    protected double velocity = 0;
    protected double wantedPosition = 0;
    protected double targetPosition = 0;
    protected boolean isHoodInPosition = false;

    public void updateInputs() {}
    public void setPosition(double position) {}

    // you don't use setVelocity nor setVoltage in any of your layers; get rid of them
    public void setVelocity(double velocity) {}
    public void setVoltage(double voltage) {}

    public void stop() {}
}
