package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.InterpolatingShotTree.ShotData;

public class GetShotData {
    public static final InterpolatingShotTree shotMap = new InterpolatingShotTree();

    static {
        shotMap.put(
            1.0,
            new ShotData(Units.degreesToRadians(45), 3.0, 3.0));
        shotMap.put(
            2.0,
            new ShotData(Units.degreesToRadians(90), 3.0, 3.0));
    }

    public static double getHoodAngle(double distance) {
        return shotMap.get(distance).hoodAngle();
    }

    public static double getRPM(double distance) {
        return shotMap.get(distance).RPM();
    }

    public static double getFlightTime(double distance) {
        return shotMap.get(distance).flightTime();
    }
}
