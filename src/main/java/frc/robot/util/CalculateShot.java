package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class CalculateShot {
    public static final InterpolatingShotTree shotMap = new InterpolatingShotTree();

    static {
        shotMap.put(
            1.0,
            new ShotData(Units.degreesToRadians(45), 3.0));
        shotMap.put(
            2.0,
            new ShotData(Units.degreesToRadians(90), 3.0));
    }
}
