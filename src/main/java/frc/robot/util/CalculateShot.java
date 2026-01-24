package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.constants.TunerConstants;

public class CalculateShot {
    public record AdjustedShot(Rotation2d targetRotation, double shootSpeed, double hoodAngle) {}

    final static int maxGoalPositionIterations = 5;
    final static double LOOKAHEAD_TIME = 0.3;
    final static double accelerationCompensation = 0.5;

    public static AdjustedShot calculateAdjustedShot(Pose2d robotPose, ChassisSpeeds fieldRelativeRobotVelocity,
            ChassisAccelerations fieldRelativeRobotAcceleration) {
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d hubTranslation = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue)
                ? FieldConstants.blueHub
                : FieldConstants.redHub;

        double distance = robotTranslation.getDistance(hubTranslation);
        double flightTime = GetShotData.getFlightTime(distance);

        Translation2d adjustedHubTranslation = new Translation2d();
        for (int i = 0; i < maxGoalPositionIterations; i++) {
            adjustedHubTranslation = new Translation2d(
                    hubTranslation.getX() - (fieldRelativeRobotVelocity.vxMetersPerSecond * flightTime
                            + 0.5 * fieldRelativeRobotAcceleration.axMetersPerSecondSquared * flightTime * flightTime * accelerationCompensation), // vt + 0.5at^2 
                    hubTranslation.getY() - (fieldRelativeRobotVelocity.vyMetersPerSecond * flightTime
                            + 0.5 * fieldRelativeRobotAcceleration.ayMetersPerSecondSquared * flightTime * flightTime * accelerationCompensation));
            double newDistance = robotTranslation.getDistance(adjustedHubTranslation);
            double newFlightTime = GetShotData.getFlightTime(newDistance);

            if (Math.abs(newFlightTime - flightTime) <= 0.01) {
                break;
            }

            flightTime = newFlightTime;
        }

        Translation2d swerveLookAheadTranslation = robotTranslation
                .plus(new Translation2d(fieldRelativeRobotVelocity.vxMetersPerSecond * LOOKAHEAD_TIME,
                        fieldRelativeRobotVelocity.vyMetersPerSecond * LOOKAHEAD_TIME));

        double adjustedDistance = swerveLookAheadTranslation.getDistance(adjustedHubTranslation);

        Rotation2d targetRotation = Rotation2d
                .fromRadians(Math.atan2(adjustedHubTranslation.getY() - robotTranslation.getY(),
                        adjustedHubTranslation.getX() - robotTranslation.getX()));
        double shootSpeed = GetShotData.getRPM(adjustedDistance);
        double hoodAngle = GetShotData.getHoodAngle(adjustedDistance);

        return new AdjustedShot(targetRotation, shootSpeed, hoodAngle);
    }
}
