package frc.robot.util;

public class ShotData {
private final double hoodAngle;
private final double RPM;

  public ShotData(
      double hoodAngle,
      double RPM) {
    this.hoodAngle = hoodAngle;
    this.RPM = RPM;
  }

  public double getHoodAngle() {
    return hoodAngle;
  }

  public double getRPM() {
    return RPM;
  }
}
