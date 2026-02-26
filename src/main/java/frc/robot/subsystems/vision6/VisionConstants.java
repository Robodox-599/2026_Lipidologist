package frc.robot.subsystems.vision6;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final double maxZError = 0.2; // 0.75 
    public static final double maxAmbiguity = 0.2; // 0.3

    public static double linearStdDevBaseline = 0.05; // Meters (0.02)
    public static double angularStdDevBaseline = 0.1; // Radians (0.06)
    public static final double singleTagMaxDistanceMeters = 5; 

    // public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
    // public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

    public record CameraConstants(String name, Transform3d robotToCamera) {
    }

    // /* VISION */
    // FRONT LEFT CAMERA
    public static final String frontLeftCameraName = "FL_Camera";

    // CAMERA 1 POSE (X)
    public static final double frontLeftCameraPoseX = Units.inchesToMeters(-11.439); // -11.439 (Y value in CAD)

    // CAMERA 1 POSE (Y)
    public static final double frontLeftCameraPoseY = Units.inchesToMeters(8.487); // -8.487 (X value in CAD)

    // CAMERA 1 POSE (Z)
    public static final double frontLeftCameraPoseZ = Units.inchesToMeters(18.451); // 18.451 (Z value in CAD)

    // CAMERA 1 POSE (ROLL)
    public static final double frontLeftCameraPoseRoll = Units.degreesToRadians(-5.9670); // 5.9670

    // CAMERA 1 POSE (PITCH)
    public static final double frontLeftCameraPosePitch = Units.degreesToRadians(60.7203-90); // 90 - 60.7203 (up/down rotation)

    // CAMERA 1 POSE (YAW)
    public static final double frontLeftCameraPoseYaw = Units.degreesToRadians(180+12); // 102.0 - 90.0 (left/right rotation)

    // FRONT RIGHT CAMERA
    public static final String frontRightCameraName = "FR_Camera";

    // CAMERA 2 POSE (X)
    public static final double frontRightCameraPoseX = Units.inchesToMeters(-11.354); // -11.354 (Y value in CAD)

    // CAMERA 2 POSE (Y)
    public static final double frontRightCameraPoseY = Units.inchesToMeters(-8.487); // -8.487 (X value in CAD)

    // CAMERA 2 POSE (Z)
    public static final double frontRightCameraPoseZ = Units.inchesToMeters(18.369); // 18.369 (Z value in CAD)

    // CAMERA 2 POSE (ROLL)
    public static final double frontRightCameraPoseRoll = Units.degreesToRadians(4.4670); // -4.4670

    // CAMERA 2 POSE (PITCH)
    public static final double frontRightCameraPosePitch = Units.degreesToRadians(68.5050-90); // 90 - 68.5050 (up/down rotation)

    // CAMERA 2 POSE (YAW)
    public static final double frontRightCameraPoseYaw = Units.degreesToRadians(180-12); // 90.0 - 102.0 (left/right rotation)

//     // BACK CAMERA
//     public static final String backCameraName = "B_Camera";

//     // CAMERA 3 POSE (X)
//     public static final double backCameraPoseX = Units.inchesToMeters(0);

//     // CAMERA 3 POSE (Z)
//     public static final double backCameraPoseZ = Units.inchesToMeters(40.93245);

//     // CAMERA 3 POSE (Y)
//     public static final double backCameraPoseY = Units.inchesToMeters(0.63967);

//     // CAMERA 3 POSE (ROLL)
//     public static final double backCameraPoseRoll = Units.degreesToRadians(0);

//     // CAMERA 3 POSE (PITCH)
//     public static final double backCameraPosePitch = Units.degreesToRadians(-33.03);

//     // CAMERA 3 POSE (YAW)
//     public static final double backCameraPoseYaw = Units.degreesToRadians(180);
    // public static final String camera4Name = "BR_Camera";

    // // CAMERA 4 POSE (X)
    // public static final double camera4PoseX = Units.inchesToMeters(12.37664406);

    // // CAMERA 4 POSE (Z)
    // public static final double camera4PoseZ = Units.inchesToMeters(4.88755783);

    // // CAMERA 4 POSE (Y)
    // public static final double camera4PoseY = Units.inchesToMeters(-7.86625496);

    // // CAMERA 4 POSE (ROLL)
    // public static final double camera4PoseRoll = Units.degreesToRadians(0);

    // // CAMERA 4 POSE (PITCH)
    // public static final double camera4PosePitch = Units.degreesToRadians(-15);

    // // CAMERA 4 POSE (YAW)
    // public static final double camera4PoseYaw = Units.degreesToRadians(-28.6588);

    public static final CameraConstants frontLeftCameraConstants = new CameraConstants(
            frontLeftCameraName,
            new Transform3d(
                    new Translation3d(frontLeftCameraPoseX, frontLeftCameraPoseY, frontLeftCameraPoseZ),
                    new Rotation3d(
                            frontLeftCameraPoseRoll, frontLeftCameraPosePitch, frontLeftCameraPoseYaw)));

    public static final CameraConstants frontRightCameraConstants = new CameraConstants(
            frontRightCameraName,
            new Transform3d(
                    new Translation3d(
                            frontRightCameraPoseX, frontRightCameraPoseY, frontRightCameraPoseZ),
                    new Rotation3d(
                            frontRightCameraPoseRoll, frontRightCameraPosePitch, frontRightCameraPoseYaw)));

    // public static final CameraConstants backCameraConstants =
    // new CameraConstants(
    // backCameraName,
    // new Transform3d(
    // new Translation3d(backCameraPoseX, backCameraPoseY, backCameraPoseZ),
    // new Rotation3d(backCameraPoseRoll, backCameraPosePitch, backCameraPoseYaw)));
    // public static final VisionConstants cam4Constants =
    // new VisionConstants(
    // camera4Name,
    // new Transform3d(
    // new Translation3d(camera4PoseX, camera4PoseY, camera4PoseZ),
    // new Rotation3d(camera4PoseRoll, camera4PosePitch, camera4PoseYaw)),
    // 1.0);
}
