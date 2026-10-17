package frc.robot.util.shootingutil;

import frc.robot.util.shootingutil.InterpolatingShotTree.Shot;

public class ShotData {
  public static final InterpolatingShotTree hubShotMap = new InterpolatingShotTree();
  public static final InterpolatingShotTree allianceZoneShotMap = new InterpolatingShotTree();
  public static final double shotMultiplier = 1.06;

  static {
    hubShotMap.put(1.553, new Shot(0.003, 29.297 * shotMultiplier, 1.14));
    hubShotMap.put(1.862, new Shot(0.005, 29.870 * shotMultiplier, 1.14));
    hubShotMap.put(2.287, new Shot(0.008, 31.244 * shotMultiplier, 1.14));
    hubShotMap.put(3.179, new Shot(0.019, 33.822 * shotMultiplier, 1.14));
    hubShotMap.put(3.535, new Shot(0.025, 34.662 * shotMultiplier, 1.14));
    hubShotMap.put(4.194, new Shot(0.035, 35.345 * shotMultiplier, 1.14));
    hubShotMap.put(4.404, new Shot(0.035, 36.5 * shotMultiplier, 1.14));
  }

  static {
    allianceZoneShotMap.put(2.5, new Shot(0.11, 15, 1));
    allianceZoneShotMap.put(16.5, new Shot(0.11, 35, 1));
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
