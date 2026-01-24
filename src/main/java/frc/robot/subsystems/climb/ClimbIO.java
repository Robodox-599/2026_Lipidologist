package frc.robot.subsystems.climb;

public abstract class ClimbIO {

  // left motor
  protected double leftMotorVelocityInchesPerSec = 0.0;
  protected double leftMotorAppliedVolts = 0.0;
  protected double leftMotorTempCelsius = 0.0;
  protected double leftMotorPositionInches = 0.0;
  protected double leftMotorTargetPositionInches = 0.0;
  protected double leftMotorStatorCurrent = 0.0;
  protected double leftMotorSupplyCurrent = 0.0;

  // right motor
  protected double rightMotorVelocityInchesPerSec = 0.0;
  protected double rightMotorAppliedVolts = 0.0;
  protected double rightMotorTempCelsius = 0.0;
  protected double rightMotorPositionInches = 0.0;
  protected double rightMotorTargetPositionInches = 0.0;
  protected double rightMotorStatorCurrent = 0.0;
  protected double rightMotorSupplyCurrent = 0.0;

  public void updateInputs() {}

  public void setLeftClimbHeight(double height) {}

  public void setRightClimbHeight(double height) {}

  public void stopClimb() {}

  public void zeroClimbPosition() {}

  public void setLeftClimbVoltage(double voltage) {}

  public void setRightClimbVoltage(double voltage) {}
}