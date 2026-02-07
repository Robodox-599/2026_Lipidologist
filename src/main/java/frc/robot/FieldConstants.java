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
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final double fieldLengthMeters = 16.541; 
  public static final double fieldWidthMeters = 8.069;

  public static class Hub {
    // Dimensions
    public static final double width = Units.inchesToMeters(47.0); 
    public static final double height =
        Units.inchesToMeters(72.0); // includes the catcher at the top
    public static final double innerWidth = Units.inchesToMeters(41.7);
    public static final double innerHeight = Units.inchesToMeters(56.5);

    // Relevant reference points on alliance side
    public static final Translation3d topCenterPoint =
        new Translation3d(
            AprilTags.getTagPose(26).getX() + width / 2.0,
            fieldWidthMeters / 2.0,
            height);
    public static final Translation3d innerCenterPoint =
        new Translation3d(
            AprilTags.getTagPose(26).getX() + width / 2.0,
            fieldWidthMeters / 2.0,
            innerHeight);

    public static final Translation2d nearLeftCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidthMeters / 2.0 + width / 2.0);
    public static final Translation2d nearRightCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidthMeters / 2.0 - width / 2.0);
    public static final Translation2d farLeftCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidthMeters / 2.0 + width / 2.0);
    public static final Translation2d farRightCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidthMeters / 2.0 - width / 2.0);

    // Relevant reference points on the opposite side
    public static final Translation3d oppTopCenterPoint =
        new Translation3d(
            AprilTags.getTagPose(4).getX() + width / 2.0,
            fieldWidthMeters / 2.0,
            height);
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidthMeters / 2.0 + width / 2.0);
    public static final Translation2d oppNearRightCorner =
        new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidthMeters / 2.0 - width / 2.0);
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidthMeters / 2.0 + width / 2.0);
    public static final Translation2d oppFarRightCorner =
        new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidthMeters / 2.0 - width / 2.0);

    // Hub faces
    public static final Pose2d nearFace =
        AprilTags.getTagPose(26).toPose2d();
    public static final Pose2d farFace =
        AprilTags.getTagPose(20).toPose2d();
    public static final Pose2d rightFace =
        AprilTags.getTagPose(18).toPose2d();
    public static final Pose2d leftFace =
        AprilTags.getTagPose(21).toPose2d();
  }

    public static class LinesVertical {
    public static final double center = fieldLengthMeters / 2.0;
    public static final double starting =
        AprilTags.getTagPose(26).getX();
    public static final double allianceZone = starting;
    public static final double hubCenter =
        AprilTags.getTagPose(26).getX() + Hub.width / 2.0;
    public static final double neutralZoneNear = center - Units.inchesToMeters(120);
    public static final double neutralZoneFar = center + Units.inchesToMeters(120);
    public static final double oppHubCenter =
        AprilTags.getTagPose(4).getX() + Hub.width / 2.0;
    public static final double oppAllianceZone =
        AprilTags.getTagPose(10).getX();
  }

  /** Left Bump related constants */
  public static class LeftBump {

    // Dimensions
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    // Relevant reference points on alliance side
    public static final Translation2d nearLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
    public static final Translation2d farLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d farRightCorner = Hub.farLeftCorner;

    // Relevant reference points on opposing side
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
  }

  /** Right Bump related constants */
  public static class RightBump {
    // Dimensions
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    // Relevant reference points on alliance side
    public static final Translation2d nearLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
    public static final Translation2d farLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d farRightCorner = Hub.farLeftCorner;

    // Relevant reference points on opposing side
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
  }

  /** Left Trench related constants */
  public static class LeftTrench {
    // Dimensions
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    // Relevant reference points on alliance side
    public static final Translation3d openingTopLeft =
        new Translation3d(LinesVertical.hubCenter, fieldWidthMeters, openingHeight);
    public static final Translation3d openingTopRight =
        new Translation3d(LinesVertical.hubCenter, fieldWidthMeters - openingWidth, openingHeight);

    // Relevant reference points on opposing side
    public static final Translation3d oppOpeningTopLeft =
        new Translation3d(LinesVertical.oppHubCenter, fieldWidthMeters, openingHeight);
    public static final Translation3d oppOpeningTopRight =
        new Translation3d(LinesVertical.oppHubCenter, fieldWidthMeters - openingWidth, openingHeight);
  }

  public static class RightTrench {

    // Dimensions
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    // Relevant reference points on alliance side
    public static final Translation3d openingTopLeft =
        new Translation3d(LinesVertical.hubCenter, openingWidth, openingHeight);
    public static final Translation3d openingTopRight =
        new Translation3d(LinesVertical.hubCenter, 0, openingHeight);

    // Relevant reference points on opposing side
    public static final Translation3d oppOpeningTopLeft =
        new Translation3d(LinesVertical.oppHubCenter, openingWidth, openingHeight);
    public static final Translation3d oppOpeningTopRight =
        new Translation3d(LinesVertical.oppHubCenter, 0, openingHeight);
  }

  /** Tower related constants */
  public static class Tower {
    // Dimensions
    public static final double width = Units.inchesToMeters(49.25);
    public static final double depth = Units.inchesToMeters(45.0);
    public static final double height = Units.inchesToMeters(78.25);
    public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
    public static final double frontFaceX = Units.inchesToMeters(43.51);

    public static final double uprightHeight = Units.inchesToMeters(72.1);

    // Rung heights from the floor
    public static final double lowRungHeight = Units.inchesToMeters(27.0);
    public static final double midRungHeight = Units.inchesToMeters(45.0);
    public static final double highRungHeight = Units.inchesToMeters(63.0);

    // Relevant reference points on alliance side
    public static final Translation2d centerPoint =
        new Translation2d(
            frontFaceX, AprilTags.getTagPose(31).getY());
    public static final Translation2d leftUpright =
        new Translation2d(
            frontFaceX,
            (AprilTags.getTagPose(31).getY())
                + innerOpeningWidth / 2
                + Units.inchesToMeters(0.75));
    public static final Translation2d rightUpright =
        new Translation2d(
            frontFaceX,
            (AprilTags.getTagPose(31).getY())
                - innerOpeningWidth / 2
                - Units.inchesToMeters(0.75));

    // Relevant reference points on opposing side
    public static final Translation2d oppCenterPoint =
        new Translation2d(
            fieldLengthMeters - frontFaceX,
            AprilTags.getTagPose(15).getY());
    public static final Translation2d oppLeftUpright =
        new Translation2d(
            fieldLengthMeters - frontFaceX,
            (AprilTags.getTagPose(15).getY())
                + innerOpeningWidth / 2
                + Units.inchesToMeters(0.75));
    public static final Translation2d oppRightUpright =
        new Translation2d(
            fieldLengthMeters - frontFaceX,
            (AprilTags.getTagPose(15).getY())
                - innerOpeningWidth / 2
                - Units.inchesToMeters(0.75));
  }

  public static class Depot {
    // Dimensions
    public static final double width = Units.inchesToMeters(42.0);
    public static final double depth = Units.inchesToMeters(27.0);
    public static final double height = Units.inchesToMeters(1.125);
    public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

    // Relevant reference points on alliance side
    public static final Translation3d depotCenter =
        new Translation3d(depth, (fieldWidthMeters / 2) + distanceFromCenterY, height);
    public static final Translation3d leftCorner =
        new Translation3d(depth, (fieldWidthMeters / 2) + distanceFromCenterY + (width / 2), height);
    public static final Translation3d rightCorner =
        new Translation3d(depth, (fieldWidthMeters / 2) + distanceFromCenterY - (width / 2), height);
  }

  public static class Outpost {
    // Dimensions
    public static final double width = Units.inchesToMeters(31.8);
    public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
    public static final double height = Units.inchesToMeters(7.0);

    // Relevant reference points on alliance side
    public static final Translation2d centerPoint =
        new Translation2d(0, AprilTags.getTagPose(29).getY());
  }

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

    public static Pose3d getTagPose(int tagID) {
        return TAGS[tagID].pose;
    }
  }
}
