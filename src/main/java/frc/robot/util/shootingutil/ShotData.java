package frc.robot.util.shootingutil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.shootingutil.InterpolatingShotTree.Shot;

public class ShotData {
    public static final InterpolatingShotTree hubShotMap = new InterpolatingShotTree();
    public static final InterpolatingShotTree allianceZoneShotMap = new InterpolatingShotTree();

    static {hubShotMap.put(
            1.41,
            new Shot(0.00, 49, 1.14));
        hubShotMap.put(
            1.83,
            new Shot(0.005, 52, 1.15));
        hubShotMap.put(
            2.24,
            new Shot(0.01, 55, 1.21));
        hubShotMap.put(
            2.645,
            new Shot(0.015, 56, 1.22));
        hubShotMap.put(
            3.000,
            new Shot(0.018, 58, 1.31));
        hubShotMap.put(
            3.404,
            new Shot(0.02, 60, 1.35));
        hubShotMap.put(
            3.803,
            new Shot(0.022, 63, 1.37));
        hubShotMap.put(
            4.215,
            new Shot(0.024, 65, 1.42));
        hubShotMap.put(
            4.63,
            new Shot(0.026, 68, 1.49));
        hubShotMap.put(
            5.04,
            new Shot(0.028, 71, 1.53));
        hubShotMap.put(
            5.39,
            new Shot(0.03, 73, 1.58));
    }

    static {
        allianceZoneShotMap.put(
            2.765,
            new Shot(0.014, 50, 1));
        allianceZoneShotMap.put(
            4.054,
            new Shot(0.022, 57, 1));
        allianceZoneShotMap.put(
            5.58,
            new Shot(0.042, 63, 1));
        allianceZoneShotMap.put(
            6.19,
            new Shot(0.048, 68, 1));
        allianceZoneShotMap.put(
            7.8,
            new Shot(0.063, 75, 1));
        allianceZoneShotMap.put(
            10.155,
            new Shot(0.08, 80, 1));
        allianceZoneShotMap.put(
            16.5,
            new Shot(0.11, 90, 1));
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
