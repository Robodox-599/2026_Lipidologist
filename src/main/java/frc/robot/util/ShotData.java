package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.InterpolatingShotTree.Shot;

public class ShotData {
    public static final InterpolatingShotTree hubShotMap = new InterpolatingShotTree();
    public static final InterpolatingShotTree allianceZoneShotMap = new InterpolatingShotTree();

    static {
        hubShotMap.put(
            0.15,
            new Shot(Units.degreesToRadians(85.6), 4000, 1.47));
        hubShotMap.put(
            0.3,
            new Shot(Units.degreesToRadians(85), 4000, 1.47));
        hubShotMap.put(
            0.6,
            new Shot(Units.degreesToRadians(83.6), 4000, 1.46));
        hubShotMap.put(
            0.9,
            new Shot(Units.degreesToRadians(81.6), 4000, 1.45));
        hubShotMap.put(
            1.2,
            new Shot(Units.degreesToRadians(80.6), 4000, 1.45));
        hubShotMap.put(
            1.5,
            new Shot(Units.degreesToRadians(78), 4000, 1.43));
        hubShotMap.put(
            2.0,
            new Shot(Units.degreesToRadians(75.3), 4000, 1.42));
        hubShotMap.put(
            2.5,
            new Shot(Units.degreesToRadians(72), 4000, 1.39));
        hubShotMap.put(
            3.0,
            new Shot(Units.degreesToRadians(68.3), 4000, 1.35));
        hubShotMap.put(
            3.5,
            new Shot(Units.degreesToRadians(64), 4000, 1.30));
        hubShotMap.put(
            4.0,
            new Shot(Units.degreesToRadians(57), 4000, 1.19));
        hubShotMap.put(
            4.5,
            new Shot(Units.degreesToRadians(59), 4300, 1.30));
        hubShotMap.put(
            5.0,
            new Shot(Units.degreesToRadians(62), 4500, 1.45));
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
