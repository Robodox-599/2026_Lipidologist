package frc.robot.util.shootingutil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.shootingutil.InterpolatingShotTree.Shot;

public class ShotData {
    public static final InterpolatingShotTree hubShotMap = new InterpolatingShotTree();
    public static final InterpolatingShotTree allianceZoneShotMap = new InterpolatingShotTree();

    static {
        hubShotMap.put(
            1.385,
            new Shot(0.00, 45, 1.02));
        hubShotMap.put(
            1.77,
            new Shot(0.005, 47, 1.04));
        hubShotMap.put(
            2.26,
            new Shot(0.01, 49, 1.1));
        hubShotMap.put(
            2.69,
            new Shot(0.015, 51, 1.13));
        hubShotMap.put(
            3.13,
            new Shot(0.018, 54, 1.21));
        hubShotMap.put(
            3.55,
            new Shot(0.02, 57, 1.24));
        hubShotMap.put(
            4.18,
            new Shot(0.023, 61, 1.32));
        hubShotMap.put(
            4.62,
            new Shot(0.025, 63, 1.33));
        hubShotMap.put(
            5.1,
            new Shot(0.028, 65, 1.34));
    }

    static {
        allianceZoneShotMap.put(
            1.0,
            new Shot(Units.degreesToRadians(45), 3.0, 3.0));
        allianceZoneShotMap.put(
            2.0,
            new Shot(Units.degreesToRadians(90), 3.0, 3.0));
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
