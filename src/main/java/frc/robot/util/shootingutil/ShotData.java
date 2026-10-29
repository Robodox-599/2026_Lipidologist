package frc.robot.util.shootingutil;

import frc.robot.util.shootingutil.InterpolatingShotTree.Shot;

public class ShotData {
  public static final InterpolatingShotTree hubShotMap = new InterpolatingShotTree();
  public static final InterpolatingShotTree allianceZoneShotMap = new InterpolatingShotTree();
  public static final double shotMultiplier = 1.00;

  static {
    hubShotMap.put(1.514, new Shot(0.007,  30.00 * shotMultiplier, 1.14));
    hubShotMap.put(1.918, new Shot(0.015,  30.00 * shotMultiplier, 1.14));
    hubShotMap.put(2.244, new Shot(0.0243, 30.00 * shotMultiplier, 1.14));
    hubShotMap.put(2.672, new Shot(0.0456, 31.50 * shotMultiplier, 1.14));
    hubShotMap.put(3.369, new Shot(0.0456, 32.50 * shotMultiplier, 1.14));
    hubShotMap.put(3.840, new Shot(0.0473, 32.50 * shotMultiplier, 1.14));
    hubShotMap.put(4.310, new Shot(0.047,  33.85 * shotMultiplier, 1.14));
    hubShotMap.put(4.741, new Shot(0.040,  34.00 * shotMultiplier, 1.14));
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
