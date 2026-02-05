package frc.robot.subsystems.climb;

public abstract class ClimbIO {

  protected double velocityInchesPerSec = 0.0;
  protected double appliedVolts = 0.0;
  protected double tempCelsius = 0.0;
  protected double positionInches = 0.0;
  protected double targetPositionInches = 0.0;
  protected double statorCurrent = 0.0;
  protected double supplyCurrent = 0.0;

  public void updateInputs() {}

  public void setClimbHeight(double height) {}

  public void stopClimb() {}

  public void zeroClimbPosition() {}

  public void setClimbVoltage(double voltage) {}
}