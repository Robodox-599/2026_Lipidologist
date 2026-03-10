package frc.robot.util.shootingutil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.shootingutil.InterpolatingShotTree.Shot;

public class ShotData {
    public static final InterpolatingShotTree hubShotMap = new InterpolatingShotTree();
    public static final InterpolatingShotTree allianceZoneShotMap = new InterpolatingShotTree();

    static {
        hubShotMap.put(
            1.41,
            new Shot(0.00, 48, 1.14));
        hubShotMap.put(
            1.83,
            new Shot(0.005, 51, 1.15));
        hubShotMap.put(
            2.24,
            new Shot(0.01, 54, 1.21));
        hubShotMap.put(
            2.645,
            new Shot(0.015, 55, 1.22));
        hubShotMap.put(
            3.000,
            new Shot(0.018, 57, 1.31));
        hubShotMap.put(
            3.404,
            new Shot(0.02, 59, 1.35));
        hubShotMap.put(
            3.803,
            new Shot(0.022, 62, 1.37));
        hubShotMap.put(
            4.215,
            new Shot(0.024, 64, 1.42));
        hubShotMap.put(
            4.63,
            new Shot(0.026, 67, 1.49));
        hubShotMap.put(
            5.04,
            new Shot(0.028, 69, 1.53));
        hubShotMap.put(
            5.39,
            new Shot(0.03, 72, 1.58));
    }

    static {
        allianceZoneShotMap.put(
            1.0,
            new Shot(0.04, 60, 1));
        allianceZoneShotMap.put(
            9.0,
            new Shot(0.07, 60, 1));
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
