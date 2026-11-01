package frc.robot.util.shootingutil;

import frc.robot.util.shootingutil.InterpolatingShotTree.Shot;

public class ShotData {
  public static final InterpolatingShotTree hubShotMap = new InterpolatingShotTree();
  public static final InterpolatingShotTree allianceZoneShotMap = new InterpolatingShotTree();
  public static final double shotMultiplier = 0.98;

  static {
    hubShotMap.put(1.514, new Shot(0.007, 29.97 * shotMultiplier, 1.14));
    hubShotMap.put(1.553, new Shot(0.003, 32.2267 * shotMultiplier, 1.14));
    hubShotMap.put(1.862, new Shot(0.005, 32.857 * shotMultiplier, 1.14));
    hubShotMap.put(2.287, new Shot(0.008, 34.3684 * shotMultiplier, 1.14));
    hubShotMap.put(3.179, new Shot(0.019, 37.2042 * shotMultiplier, 1.14));
    hubShotMap.put(3.535, new Shot(0.025, 38.0842 * shotMultiplier, 1.14));
    hubShotMap.put(4.194, new Shot(0.035, 38.8795 * shotMultiplier, 1.14));
    hubShotMap.put(4.404, new Shot(0.035, 40.15 * shotMultiplier, 1.14));
  }

  // hubShotMap.put(1.918, new Shot(0.015, 30.00 * shotMultiplier, 0.75));
  // hubShotMap.put(2.244, new Shot(0.0243, 30.00 * shotMultiplier, 0.780));
  // hubShotMap.put(2.672, new Shot(0.0455, 31.425 * shotMultiplier, 0.980));
  // hubShotMap.put(3.369, new Shot(0.0455, 32.45 * shotMultiplier, 1.1));
  // hubShotMap.put(3.840, new Shot(0.0473, 32.53 * shotMultiplier, 1.18));
  // hubShotMap.put(4.310, new Shot(0.047, 33.85 * shotMultiplier, 1.25));
  // hubShotMap.put(4.741, new Shot(0.0493, 37.5 * shotMultiplier, 1.380));

  static {
    allianceZoneShotMap.put(2.5, new Shot(0.08, 30.0, 1));
    allianceZoneShotMap.put(6.0, new Shot(0.08, 36.0, 1));
    allianceZoneShotMap.put(10.0, new Shot(0.08, 40.0, 1));
    allianceZoneShotMap.put(14.0, new Shot(0.08, 45.0, 1));
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
