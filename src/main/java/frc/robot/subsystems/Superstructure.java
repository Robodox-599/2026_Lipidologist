package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.util.CalculateShot;
import frc.robot.subsystems.shooter.flywheels.Flywheels;

public class Superstructure extends SubsystemBase {

  public record AdjustedShot(Rotation2d targetRotation, double shootSpeed, double hoodAngle) {}  
  private static final double LOOKAHEAD_TIME = 0.3;

    private final CommandXboxController driver;
    private final CommandSwerveDrivetrain drivetrain;
    // private final Intake intake;
    // private final Feeder feeder;
    // private final Indexer indexer;
    // private final Hood hood;
    // private final Flywheels flywheels;
    private final CalculateShot shotCalculator = new CalculateShot();

    public enum WantedSuperState {
        PREPARE_HUB_SHOT,
        SHOOT_HUB,
        PREPARE_ALLIANCE_ZONE_SHOT,
        SHOOT_ALLIANCE_ZONE,
        IDLE,
        STOP,
    }

    public enum CurrentSuperState {
        PREPARING_HUB_SHOT,
        SHOOTING_HUB,
        PREPARING_ALLIANCE_ZONE_SHOT,
        SHOOTING_ALLIANCE_ZONE,
        IDLING,
        STOPPED;
    }

    public enum AutomationLevel {
        AUTO_ROTATION_LOCK,
        AUTO_SHOOT,
        MANUAL;
    }

    private WantedSuperState wantedSuperState = WantedSuperState.STOP;
    private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
    private AutomationLevel automationLevel = AutomationLevel.AUTO_SHOOT;

    public Superstructure(
        CommandXboxController driver,
        CommandSwerveDrivetrain drivetrain
        // Intake intake,
        // Feeder feeder, 
        // Indexer indexer,
        // Hood hood,
        // Flywheels flywheels
        ) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        // this.intake = intake;
        // this.feeder = feeder;
        // this.indexer = indexer;
        // this.hood = hood;
        // this.flywheels = flywheels;
    }

    @Override
    public void periodic() {
        drivetrain.updateInputs();
        // This method will be called once per scheduler run
    }

    private void handleStateTransitions() {
        switch (wantedSuperState) {
            default:
            case PREPARE_HUB_SHOT:
                if (automationLevel == AutomationLevel.AUTO_SHOOT) {
                    if (areSystemsReadyForShot()) {
                        wantedSuperState = WantedSuperState.SHOOT_HUB;
                        currentSuperState = CurrentSuperState.SHOOTING_HUB;
                    } else {
                        currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT;
                    }
                } else {
                    currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT;
                }
                break;
            case SHOOT_HUB:
            case PREPARE_ALLIANCE_ZONE_SHOT:
            case SHOOT_ALLIANCE_ZONE:
            case IDLE:
            case STOP:

        }
    }

    public void preparingHubShot() {
        AdjustedShot adjustedShot = calculatedAdjustedShot();

        drivetrain.setTargetRotation(adjustedShot.targetRotation());
        
    }

    public AdjustedShot calculatedAdjustedShot() {
        Pose2d pose = drivetrain.getPose();
        ChassisSpeeds currentChassisSpeeds = drivetrain.getChassisSpeeds();

        Translation2d currentTranslation = pose.getTranslation();
        Translation2d hubTranslation = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue) ? FieldConstants.blueHub : FieldConstants.redHub;

        ChassisSpeeds controllerChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-drivetrain.joystickDeadbandApply(driver.getLeftY())
                        * TunerConstants.MaxSpeed, -drivetrain.joystickDeadbandApply(driver.getLeftX())
                        * TunerConstants.MaxSpeed, 0.0, pose.getRotation());

        ChassisSpeeds averageChassisSpeeds = new ChassisSpeeds((controllerChassisSpeeds.vxMetersPerSecond + currentChassisSpeeds.vxMetersPerSecond) / 2.0, (controllerChassisSpeeds.vyMetersPerSecond + currentChassisSpeeds.vyMetersPerSecond) / 2.0, 0.0);
        // average the controller and robot chassis speeds in order to account for robot speed and what the controller is commanding

        double distance = currentTranslation.getDistance(hubTranslation);

        double flightTime = shotCalculator.getFlightTime(distance);

        Translation2d adjustedHubTranslation = new Translation2d(hubTranslation.getX() - (averageChassisSpeeds.vxMetersPerSecond * flightTime), hubTranslation.getY() - (averageChassisSpeeds.vyMetersPerSecond * flightTime));
        // flight time multiplied by the robot's x or y velocity = the distance compensation needed 
        // that compensation is subtracted from the hub's actual position

        Translation2d swerveLookAheadTranslation = currentTranslation.plus(new Translation2d(averageChassisSpeeds.vxMetersPerSecond * LOOKAHEAD_TIME, averageChassisSpeeds.vyMetersPerSecond * LOOKAHEAD_TIME));

        double adjustedDistance = currentTranslation.getDistance(adjustedHubTranslation);

        Rotation2d targetRotation = Rotation2d.fromRadians(Math.atan2(adjustedHubTranslation.getY() - swerveLookAheadTranslation.getY(), adjustedHubTranslation.getX() - swerveLookAheadTranslation.getX()));
        double shootSpeed = shotCalculator.getRPM(adjustedDistance);
        double hoodAngle = shotCalculator.getHoodAngle(adjustedDistance);

        return new AdjustedShot(targetRotation, shootSpeed, hoodAngle);
    }

    private boolean areSystemsReadyForShot() {
        // flywheels.flywheelsAtSetpoint() && hood.hoodAtSetpoint() && swerve.isAimed
        return false;
    }

    public Command zeroGyroCommand() {
    return this.runOnce(() -> drivetrain.zeroGyro());
  }
    
}
