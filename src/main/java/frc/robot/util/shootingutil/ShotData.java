package frc.robot.util.shootingutil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.shootingutil.InterpolatingShotTree.Shot;

public class ShotData {
    public static final InterpolatingShotTree hubShotMap = new InterpolatingShotTree();
    public static final InterpolatingShotTree allianceZoneShotMap = new InterpolatingShotTree();

    static {
        hubShotMap.put(
            1.398,
            new Shot(0.00, 51, 1.17));
        hubShotMap.put(
            1.83,
            new Shot(0.005, 54, 1.20));
        hubShotMap.put(
            2.19,
            new Shot(0.008, 56, 1.24));
        hubShotMap.put(
            2.61,
            new Shot(0.013, 58, 1.31));
        hubShotMap.put(
            3.03,
            new Shot(0.018, 60, 1.34));
        hubShotMap.put(
            3.404,
            new Shot(0.023, 62, 1.36));
        hubShotMap.put(
            3.83,
            new Shot(0.027, 64, 1.4));
        hubShotMap.put(
            4.21,
            new Shot(0.03, 66, 1.44));
        hubShotMap.put(
            4.62,
            new Shot(0.032, 68, 1.45));
        hubShotMap.put(
            5.09,
            new Shot(0.035, 70, 1.46));
        hubShotMap.put(
            5.76,
            new Shot(0.04, 76, 1.56));
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
