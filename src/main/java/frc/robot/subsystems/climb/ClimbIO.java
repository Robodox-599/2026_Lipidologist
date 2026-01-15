package frc.robot.subsystems.climb;

public abstract class ClimbIO {

  protected double velocityInchesPerSec = 0.0;
  protected double appliedVolts = 0.0;
  protected double tempCelsius = 0.0;
  protected double positionInches = 0.0;
  protected double targetPositionInches = 0.0;
  protected double statorCurrent = 0.0;
  protected double supplyCurrent = 0.0;

  public void updateClimbInputs() {}

  /** Sets the target height of the elevator */
  public void setLeftClimbHeight(double height) {}

  public void setRightClimbHeight(double height) {}

  public void stopClimb() {}

  public void zeroClimbEncoder() {}

  public void setLeftClimbVoltage(double voltage) {}

  public void setRightClimbVoltage(double voltage) {}
}