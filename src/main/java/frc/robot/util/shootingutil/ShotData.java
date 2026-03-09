package frc.robot.util.shootingutil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.shootingutil.InterpolatingShotTree.Shot;

public class ShotData {
    public static final InterpolatingShotTree hubShotMap = new InterpolatingShotTree();
    public static final InterpolatingShotTree allianceZoneShotMap = new InterpolatingShotTree();

    static {
        hubShotMap.put(
            1.430,
            new Shot(0.00, 47, 0.97));
        hubShotMap.put(
            1.825,
            new Shot(0.005, 50, 1.04));
        hubShotMap.put(
            2.210,
            new Shot(0.008, 53, 1.14));
        hubShotMap.put(
            2.605,
            new Shot(0.01, 55, 1.2));
        hubShotMap.put(
            3.050,
            new Shot(0.013, 57, 1.25));
        hubShotMap.put(
            3.520,
            new Shot(0.016, 61, 1.33));
        hubShotMap.put(
            4.050,
            new Shot(0.02, 64, 1.37));
        hubShotMap.put(
            4.620,
            new Shot(0.023, 67, 1.39));
        hubShotMap.put(
            5.464,
            new Shot(0.024, 74, 1.48));
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
