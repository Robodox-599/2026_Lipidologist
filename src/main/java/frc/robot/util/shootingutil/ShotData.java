package frc.robot.util.shootingutil;

import frc.robot.util.shootingutil.InterpolatingShotTree.Shot;

public class ShotData {
  public static final InterpolatingShotTree hubShotMap = new InterpolatingShotTree();
  public static final InterpolatingShotTree allianceZoneShotMap = new InterpolatingShotTree();
  public static final double shotMultiplier = 1.00;

  static {
    hubShotMap.put(1.514, new Shot(0.690, 28.20 * shotMultiplier, 1.14));
    hubShotMap.put(1.862, new Shot(0.795, 28.80 * shotMultiplier, 1.20));
    hubShotMap.put(2.244, new Shot(0.850, 29.80 * shotMultiplier, 1.28)); 
    hubShotMap.put(2.672, new Shot(1.060, 31.42 * shotMultiplier, 1.35));
    hubShotMap.put(3.369, new Shot(1.190, 32.45 * shotMultiplier, 1.42));
    hubShotMap.put(3.840, new Shot(1.290, 33.15 * shotMultiplier, 1.48));
    hubShotMap.put(4.310, new Shot(1.390, 34.15 * shotMultiplier, 1.55));
    hubShotMap.put(4.741, new Shot(1.530, 35.10 * shotMultiplier, 1.62));
  }

  static {
    allianceZoneShotMap.put(2.5, new Shot(0.08, 25.0, 1));
    allianceZoneShotMap.put(6.0, new Shot(0.06, 32.0, 1));
    allianceZoneShotMap.put(10.0, new Shot(0.04, 38.0, 1));
    allianceZoneShotMap.put(14.0, new Shot(0.02, 45.0, 1));
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
