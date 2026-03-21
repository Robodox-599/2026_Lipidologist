package frc.robot.util.shootingutil;

import java.lang.reflect.Field;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ChassisAccelerations;

public class CalculateShot {
    public record AdjustedShot(Rotation2d targetRotation, double shootSpeed, double hoodAngle, double flightTime) {}

    final static int maxGoalPositionIterations = 5;
    final static double LOOKAHEAD_TIME = 0.05;
    final static double fuelDragCoefficient = 0.1;

    public static AdjustedShot calculateHubAdjustedShot(Pose2d robotPose, ChassisSpeeds fieldRelativeRobotVelocity,
            ChassisAccelerations fieldRelativeRobotAcceleration) {
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d hubTranslation = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

        double distance = robotTranslation.getDistance(hubTranslation);
        double flightTime = applyFuelDragCoefficient(ShotData.getHubFlightTime(distance));

        Translation2d adjustedHubTranslation = new Translation2d();
        for (int i = 0; i < maxGoalPositionIterations; i++) {
            adjustedHubTranslation = new Translation2d(
                    hubTranslation.getX() - (fieldRelativeRobotVelocity.vxMetersPerSecond * flightTime), // vt + 0.5at^2, acceleration compensation may not be needed
                    hubTranslation.getY() - (fieldRelativeRobotVelocity.vyMetersPerSecond * flightTime));
            double newDistance = robotTranslation.getDistance(adjustedHubTranslation);
            double newFlightTime = applyFuelDragCoefficient(ShotData.getHubFlightTime(newDistance));

            if (Math.abs(newFlightTime - flightTime) <= 0.01) {
                flightTime = newFlightTime;
                break;
            }
            flightTime = newFlightTime;
        }

        /* Accounts for shooting latency */
        Translation2d swerveLookAheadTranslation = robotTranslation
                .plus(new Translation2d(fieldRelativeRobotVelocity.vxMetersPerSecond * LOOKAHEAD_TIME,
                        fieldRelativeRobotVelocity.vyMetersPerSecond * LOOKAHEAD_TIME));

        double adjustedDistance = swerveLookAheadTranslation.getDistance(adjustedHubTranslation);

        // double adjustedDistance = robotTranslation.getDistance(adjustedHubTranslation);

        Rotation2d targetRotation = Rotation2d
                .fromRadians(Math.atan2(robotTranslation.getY() - adjustedHubTranslation.getY(),
                        robotTranslation.getX() - adjustedHubTranslation.getX()));
        double shootSpeed = ShotData.getHubRPS(adjustedDistance);
        double hoodAngle = ShotData.getHubHoodAngle(adjustedDistance);

        AdjustedShot adjustedShot = new AdjustedShot(targetRotation, shootSpeed, hoodAngle, flightTime);
        DogLog.log("ShotCalculator/AdjustedShot", adjustedShot);
        DogLog.log("ShotCalculator/AdjustedHubTarget", new Pose2d(adjustedHubTranslation.getX(), adjustedHubTranslation.getY(), new Rotation2d()));
        return adjustedShot;
    }

    public static AdjustedShot calculateAllianceZoneAdjustedShot(Pose2d robotPose, ChassisSpeeds fieldRelativeRobotVelocity,
            ChassisAccelerations fieldRelativeRobotAcceleration) {
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d allianceZoneTarget = new Translation2d(AllianceFlipUtil.applyX(FieldConstants.LinesVertical.starting - 1), robotTranslation.getY());

        double distance = robotTranslation.getDistance(allianceZoneTarget);

        Rotation2d targetRotation = Rotation2d
                .fromRadians(Math.atan2(robotTranslation.getY()- allianceZoneTarget.getY(),
                        robotTranslation.getX() - allianceZoneTarget.getX()));
        double shootSpeed = ShotData.getAllianceZoneRPS(distance);
        double hoodAngle = ShotData.getAllianceZoneHoodAngle(distance);
        double flightTime = ShotData.getAllianceZoneFlightTime(distance);

        AdjustedShot adjustedShot = new AdjustedShot(targetRotation, shootSpeed, hoodAngle, flightTime);
        DogLog.log("ShotCalculator/AdjustedShot", adjustedShot);
        DogLog.log("ShotCalculator/AllianceZoneTarget", new Pose2d(allianceZoneTarget.getX(), allianceZoneTarget.getY(), new Rotation2d()));
        return adjustedShot;
    }

    public static AdjustedShot calculateManualShot() {
        Translation2d robotTranslation = AllianceFlipUtil.apply(new Translation2d(3.57, 4.07));
        Translation2d hubTranslation = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

        double distance = robotTranslation.getDistance(hubTranslation);
        double flightTime = applyFuelDragCoefficient(ShotData.getHubFlightTime(distance));

        // Translation2d adjustedHubTranslation = new Translation2d();
        // for (int i = 0; i < maxGoalPositionIterations; i++) {
        //     adjustedHubTranslation = new Translation2d(
        //             hubTranslation.getX() - (fieldRelativeRobotVelocity.vxMetersPerSecond * flightTime), // vt + 0.5at^2, acceleration compensation may not be needed
        //             hubTranslation.getY() - (fieldRelativeRobotVelocity.vyMetersPerSecond * flightTime));
        //     double newDistance = robotTranslation.getDistance(adjustedHubTranslation);
        //     double newFlightTime = applyFuelDragCoefficient(ShotData.getHubFlightTime(newDistance));

        //     if (Math.abs(newFlightTime - flightTime) <= 0.01) {
        //         flightTime = newFlightTime;
        //         break;
        //     }
        //     flightTime = newFlightTime;
        // }

        /* Accounts for shooting latency */
        // Translation2d swerveLookAheadTranslation = robotTranslation
        //         .plus(new Translation2d(fieldRelativeRobotVelocity.vxMetersPerSecond * LOOKAHEAD_TIME,
        //                 fieldRelativeRobotVelocity.vyMetersPerSecond * LOOKAHEAD_TIME));

        // double adjustedDistance = swerveLookAheadTranslation.getDistance(adjustedHubTranslation);

        // double adjustedDistance = robotTranslation.getDistance(hubTranslation);

        Rotation2d targetRotation = Rotation2d
                .fromRadians(Math.atan2(robotTranslation.getY() - hubTranslation.getY(),
                        robotTranslation.getX() - hubTranslation.getX()));
        double shootSpeed = ShotData.getHubRPS(distance);
        double hoodAngle = ShotData.getHubHoodAngle(distance);

        AdjustedShot adjustedShot = new AdjustedShot(targetRotation, shootSpeed, hoodAngle, flightTime);
        DogLog.log("ShotCalculator/AdjustedShot", adjustedShot);
        DogLog.log("ShotCalculator/AdjustedHubTarget", new Pose2d(hubTranslation.getX(), hubTranslation.getY(), new Rotation2d()));
        return adjustedShot;
    }

    public static double applyFuelDragCoefficient(double flightTime) {
        return (1-Math.pow(Math.E, -(fuelDragCoefficient * flightTime))) / fuelDragCoefficient;
        // return flightTime;
    }
}
