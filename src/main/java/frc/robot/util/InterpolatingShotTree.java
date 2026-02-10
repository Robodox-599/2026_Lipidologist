package frc.robot.util;

import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class InterpolatingShotTree {
    private final TreeMap<Double, Shot> map = new TreeMap<>();

  public record Shot(double hoodAngle, double RPM, double flightTime) {}

  public InterpolatingShotTree() {}

  /**
   * This method puts shooting data points into the treemap
   * 
   * @param key the distance from the goal
   * @param value the shot's hood angle and RPM
   * 
   */
  public void put(Double distance, Shot data) {
    map.put(distance, data);
  }


  public Shot get(Double distance) {
    Shot val = map.get(distance);
    if (val == null) {
      Double ceilingKey = map.ceilingKey(distance);
      Double floorKey = map.floorKey(distance);

      if (ceilingKey == null && floorKey == null) {
        return null;
      }
      if (ceilingKey == null) {
        return map.get(floorKey);
      }
      if (floorKey == null) {
        return map.get(ceilingKey);
      }
      Shot floor = map.get(floorKey);
      Shot ceiling = map.get(ceilingKey);

      return interpolate(floor, ceiling, inverseInterpolate(ceilingKey, distance, floorKey));
    } else {
      return val;
    }
  }

  private Shot interpolate(Shot startValue, Shot endValue, double t) {
    return new Shot(
            MathUtil.interpolate(
                startValue.hoodAngle(), endValue.hoodAngle(), t),
        MathUtil.interpolate(startValue.RPM(), endValue.RPM(), t), 
        MathUtil.interpolate(startValue.flightTime(), endValue.flightTime(), t));
  }

  private double inverseInterpolate(Double upper, Double query, Double lower) {
    double delta = upper.doubleValue() - lower.doubleValue();
    double deltaQuery = query.doubleValue() - lower.doubleValue();

    /*
     * if (delta <= 0) {
      return 0.0;
    }
    if (deltaQuery <= 0) {
      return 0.0;
    }
     */
    return deltaQuery / delta;
  }

  /* 
   * public void clear() {
    map.clear();
  }

  public void remove(double key) {
    map.remove(key);
  }

  public double maxKey() {
    return map.lastKey();
  }
   */
}
