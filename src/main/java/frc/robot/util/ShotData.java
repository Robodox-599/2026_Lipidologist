package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.InterpolatingShotTree.Shot;

public class ShotData {
    public static final InterpolatingShotTree hubShotMap = new InterpolatingShotTree();
    public static final InterpolatingShotTree allianceZoneShotMap = new InterpolatingShotTree();

    static {
        hubShotMap.put(
            1.0,
            new Shot(Units.degreesToRadians(45), 4000, 0.5));
        hubShotMap.put(
            7.0,
            new Shot(Units.degreesToRadians(50), 4800, 1.5));
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

    public static double getHubRPM(double distance) {
        return hubShotMap.get(distance).RPM();
    }

    public static double getHubFlightTime(double distance) {
        return hubShotMap.get(distance).flightTime();
    }

    public static double getAllianceZoneHoodAngle(double distance) {
        return allianceZoneShotMap.get(distance).hoodAngle();
    }

    public static double getAllianceZoneRPM(double distance) {
        return allianceZoneShotMap.get(distance).RPM();
    }

    public static double getAllianceZoneFlightTime(double distance) {
        return allianceZoneShotMap.get(distance).flightTime();
    }
}
