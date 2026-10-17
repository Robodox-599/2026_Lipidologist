package frc.robot.subsystems.shooter.hood;

public class HoodIO {
  protected double positionRotations = 0;
  protected double RPS = 0;
  protected double targetPositionRots = 0;
  protected double statorCurrent = 0;
  protected double supplyCurrent = 0;
  protected double temperature = 0;
  protected boolean isHoodInPosition = false;

  public void updateInputs() {}

  public void setHoodPosition(double position) {}

  public void setHoodVoltage(double voltage) {}

  public void stopHood() {}
}
