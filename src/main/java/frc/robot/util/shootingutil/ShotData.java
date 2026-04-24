package frc.robot.util.shootingutil;

import frc.robot.util.shootingutil.InterpolatingShotTree.Shot;

public class ShotData {
  public static final InterpolatingShotTree hubShotMap = new InterpolatingShotTree();
  public static final InterpolatingShotTree allianceZoneShotMap = new InterpolatingShotTree();

  static {
    hubShotMap.put(1.5, new Shot(0.00, 28, 1.14));
    hubShotMap.put(1.79, new Shot(0.005, 30, 1.15));
    hubShotMap.put(2.21, new Shot(0.008, 31, 1.21));
    hubShotMap.put(2.59, new Shot(0.011, 32, 1.22));
    hubShotMap.put(3.06, new Shot(0.015, 34, 1.31));
    hubShotMap.put(3.41, new Shot(0.018, 36, 1.35));
    hubShotMap.put(3.88, new Shot(0.023, 37, 1.37));
    hubShotMap.put(4.3, new Shot(0.025, 39, 1.42));
  }

  static {
    allianceZoneShotMap.put(2.765, new Shot(0.014, 50, 1));
    allianceZoneShotMap.put(4.054, new Shot(0.022, 57, 1));
    allianceZoneShotMap.put(5.58, new Shot(0.042, 63, 1));
    allianceZoneShotMap.put(6.19, new Shot(0.048, 68, 1));
    allianceZoneShotMap.put(7.8, new Shot(0.063, 75, 1));
    allianceZoneShotMap.put(10.155, new Shot(0.08, 80, 1));
    allianceZoneShotMap.put(16.5, new Shot(0.11, 90, 1));
  }

  public static double getHubHoodAngle(double distance) {
    return hubShotMap.get(distance).hoodAngle();
  }

  public static double getHubRPS(double distance) {
    return hubShotMap.get(distance).RPS();
  }

  public static double getHubFlightTime(double distance) {
    return hubShotMap.get(distance).flightTime();
  }

  public static double getAllianceZoneHoodAngle(double distance) {
    return allianceZoneShotMap.get(distance).hoodAngle();
  }

  public static double getAllianceZoneRPS(double distance) {
    return allianceZoneShotMap.get(distance).RPS();
  }

  public static double getAllianceZoneFlightTime(double distance) {
    return allianceZoneShotMap.get(distance).flightTime();
  }
}
