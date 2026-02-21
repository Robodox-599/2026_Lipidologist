package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.InterpolatingShotTree.Shot;

public class ShotData {
    public static final InterpolatingShotTree hubShotMap = new InterpolatingShotTree();
    public static final InterpolatingShotTree allianceZoneShotMap = new InterpolatingShotTree();

    static {
        hubShotMap.put(
            5.0,
            new Shot(0.01, 65, 1.47));
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
