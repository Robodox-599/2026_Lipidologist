package frc.robot;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;

public class FieldConstants {
  public static final double fieldLengthMeters = 16.541; 
  public static final double fieldWidthMeters = 8.069;

  public static final Translation2d blueHub = new Translation2d(4.626, 4.035);
  public static final Translation2d redHub = new Translation2d(11.915, 4.035);

  public class AprilTags {
    public static final AprilTag[] TAGS =
    new AprilTag[] {
        new AprilTag(
            1,
            new Pose3d(
                new Translation3d(11.8779798, 7.4247756, 0.889),
                new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0))
            )
        ),
        new AprilTag(
            2,
            new Pose3d(
                new Translation3d(11.9154194, 4.638039999999999, 1.12395),
                new Rotation3d(new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476))
            )
        ),
        new AprilTag(
            3,
            new Pose3d(
                new Translation3d(11.3118646, 4.3902376, 1.12395),
                new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0))
            )
        ),
        new AprilTag(
            4,
            new Pose3d(
                new Translation3d(11.3118646, 4.0346376, 1.12395),
                new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0))
            )
        ),
        new AprilTag(
            5,
            new Pose3d(
                new Translation3d(11.9154194, 3.4312351999999997, 1.12395),
                new Rotation3d(new Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865476))
            )
        ),
        new AprilTag(
            6,
            new Pose3d(
                new Translation3d(11.8779798, 0.6444996, 0.889),
                new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0))
            )
        ),
        new AprilTag(
            7,
            new Pose3d(
                new Translation3d(11.9528844, 0.6444996, 0.889),
                new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))
            )
        ),
        new AprilTag(
            8,
            new Pose3d(
                new Translation3d(12.2710194, 3.4312351999999997, 1.12395),
                new Rotation3d(new Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865476))
            )
        ),
        new AprilTag(
            9,
            new Pose3d(
                new Translation3d(12.519177399999998, 3.6790375999999996, 1.12395),
                new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))
            )
        ),
        new AprilTag(
            10,
            new Pose3d(
                new Translation3d(12.519177399999998, 4.0346376, 1.12395),
                new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))
            )
        ),
        new AprilTag(
            11,
            new Pose3d(
                new Translation3d(12.2710194, 4.638039999999999, 1.12395),
                new Rotation3d(new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476))
            )
        ),
        new AprilTag(
            12,
            new Pose3d(
                new Translation3d(11.9528844, 7.4247756, 0.889),
                new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))
            )
        ),
        new AprilTag(
            13,
            new Pose3d(
                new Translation3d(16.5333172, 7.4033126, 0.55245),
                new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0))
            )
        ),
        new AprilTag(
            14,
            new Pose3d(
                new Translation3d(16.5333172, 6.9715126, 0.55245),
                new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0))
            )
        ),
        new AprilTag(
            15,
            new Pose3d(
                new Translation3d(16.5329616, 4.3235626, 0.55245),
                new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0))
            )
        ),
        new AprilTag(
            16,
            new Pose3d(
                new Translation3d(16.5329616, 3.8917626, 0.55245),
                new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0))
            )
        ),
        new AprilTag(
            17,
            new Pose3d(
                new Translation3d(4.6630844, 0.6444996, 0.889),
                new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))
            )
        ),
        new AprilTag(
            18,
            new Pose3d(
                new Translation3d(4.6256194, 3.4312351999999997, 1.12395),
                new Rotation3d(new Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865476))
            )
        ),
        new AprilTag(
            19,
            new Pose3d(
                new Translation3d(5.229174199999999, 3.6790375999999996, 1.12395),
                new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))
            )
        ),
        new AprilTag(
            20,
            new Pose3d(
                new Translation3d(5.229174199999999, 4.0346376, 1.12395),
                new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))
            )
        ),
        new AprilTag(
            21,
            new Pose3d(
                new Translation3d(4.6256194, 4.638039999999999, 1.12395),
                new Rotation3d(new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476))
            )
        ),
        new AprilTag(
            22,
            new Pose3d(
                new Translation3d(4.6630844, 7.4247756, 0.889),
                new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))
            )
        ),
        new AprilTag(
            23,
            new Pose3d(
                new Translation3d(4.5881798, 7.4247756, 0.889),
                new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0))
            )
        ),
        new AprilTag(
            24,
            new Pose3d(
                new Translation3d(4.2700194, 4.638039999999999, 1.12395),
                new Rotation3d(new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476))
            )
        ),
        new AprilTag(
            25,
            new Pose3d(
                new Translation3d(4.0218614, 4.3902376, 1.12395),
                new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0))
            )
        ),
        new AprilTag(
            26,
            new Pose3d(
                new Translation3d(4.0218614, 4.0346376, 1.12395),
                new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0))
            )
        ),
        new AprilTag(
            27,
            new Pose3d(
                new Translation3d(4.2700194, 3.4312351999999997, 1.12395),
                new Rotation3d(new Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865476))
            )
        ),
        new AprilTag(
            28,
            new Pose3d(
                new Translation3d(4.5881798, 0.6444996, 0.889),
                new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0))
            )
        ),
        new AprilTag(
            29,
            new Pose3d(
                new Translation3d(0.0077469999999999995, 0.6659626, 0.55245),
                new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))
            )
        ),
        new AprilTag(
            30,
            new Pose3d(
                new Translation3d(0.0077469999999999995, 1.0977626, 0.55245),
                new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))
            )
        ),
        new AprilTag(
            31,
            new Pose3d(
                new Translation3d(0.0080772, 3.7457125999999996, 0.55245),
                new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))
            )
        ),
        new AprilTag(
            32,
            new Pose3d(
                new Translation3d(0.0080772, 4.1775126, 0.55245),
                new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))
            )
        )
    };

    public static final Pose2d[] TAGS_POSE2D;

    static {
      TAGS_POSE2D = new Pose2d[TAGS.length];
      for (int i = 0; i < TAGS.length; i++) {
        TAGS_POSE2D[i] =
            new Pose2d(
                TAGS[i].pose.getTranslation().toTranslation2d(),
                TAGS[i].pose.getRotation().toRotation2d());
      }
    }

    public static final AprilTagFieldLayout aprilTagFieldLayout =
        new AprilTagFieldLayout(
            List.of(TAGS), FieldConstants.fieldLengthMeters, FieldConstants.fieldWidthMeters);
  }
}
