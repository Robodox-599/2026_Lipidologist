package frc.robot.subsystems;

import dev.doglog.DogLog;
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
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.util.CalculateShot;
import frc.robot.util.GetShotData;
import frc.robot.util.CalculateShot.AdjustedShot;
import frc.robot.subsystems.shooter.flywheels.Flywheels;

public class Superstructure extends SubsystemBase {

    private final CommandXboxController driver;
    private final CommandSwerveDrivetrain drivetrain;
    private final IntakeRollers intakeRollers;
    // private final Feeder feeder;
    // private final Indexer indexer;
    // private final Hood hood;
    // private final Flywheels flywheels;
    private final GetShotData shotCalculator = new GetShotData();

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
        CommandSwerveDrivetrain drivetrain,
        IntakeRollers intakeRollers
        // Feeder feeder, 
        // Indexer indexer,
        // Hood hood,
        // Flywheels flywheels
        ) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        this.intakeRollers = intakeRollers;
        // this.feeder = feeder;
        // this.indexer = indexer;
        // this.hood = hood;
        // this.flywheels = flywheels;
    }

    @Override
    public void periodic() {
        drivetrain.updateInputs();
        intakeRollers.updateInputs();

        handleStateTransitions();
        applyStates();
        // This method will be called once per scheduler run
        DogLog.log("Superstructure/WantedSuperState", wantedSuperState);
        DogLog.log("Superstructure/CurrentSuperState", currentSuperState);

        DogLog.log("working", true);
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
                currentSuperState = CurrentSuperState.IDLING;
                break;
            case STOP:
                currentSuperState = CurrentSuperState.STOPPED;
                break;
        }
    }

    private void applyStates() {
        switch (currentSuperState) {
            case IDLING:
                idling();
                break;
            case STOPPED:
                stop();
                break;
            default:
                stop();
                break;
        }
    }

    public void idling() {
        intakeRollers.setWantedState(IntakeRollers.WantedState.INTAKING_FUEL);
    }

    public void stop() {
        intakeRollers.setWantedState(IntakeRollers.WantedState.STOPPED);
    }

    public void preparingHubShot() {
        AdjustedShot adjustedShot = CalculateShot.calculateAdjustedShot(drivetrain.getPose(), drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());

        drivetrain.setTargetRotation(adjustedShot.targetRotation());
        // flywheels.setFlywheelWantedState(Flywheels.FlywheelWantedState.SET_VELOCITY, adjustedShot.shootSpeed());
        
    }


    private boolean areSystemsReadyForShot() {
        // flywheels.flywheelsAtSetpoint() && hood.hoodAtSetpoint() && swerve.isAimed
        return false;
    }

    public Command zeroGyroCommand() {
    return this.runOnce(() -> drivetrain.zeroGyro());
  }

  public Command setWantedSuperStateCommand(WantedSuperState wantedState) {
    return this.runOnce(() -> setWantedSuperState(wantedState));
  }

  public void setWantedSuperState(WantedSuperState state) {
    wantedSuperState = state;
  } 
}
