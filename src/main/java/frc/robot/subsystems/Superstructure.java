package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.flywheels.Flywheels;

public class Superstructure extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final Intake intake;
    private final Feeder feeder;
    private final Indexer indexer;
    private final Hood hood;
    private final Flywheels flywheels;

    public enum WantedSuperState {
        PREPARE_HUB_SHOT,
        SHOOT_HUB,
        PREPARE_ALLIANCE_ZONE_SHOT,
        SHOOT_ALLIANCE_ZONE,
        STOPPED,
    }

    public enum CurrentSuperState {
        PREPARING_HUB_SHOT,
        SHOOTING_HUB,
        PREPARING_ALLIANCE_ZONE_SHOT,
        SHOOTING_ALLIANCE_ZONE,
        STOPPED;
    }

    public enum AutomationLevel {
        AUTO_ROTATION_LOCK,
        AUTO_SHOOT,
        MANUAL;
    }

    private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
    private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
    private AutomationLevel automationLevel = AutomationLevel.AUTO_SHOOT;

    public Superstructure(
        CommandSwerveDrivetrain drivetrain,
        Intake intake,
        Feeder feeder, 
        Indexer indexer,
        Hood hood,
        Flywheels flywheels) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.feeder = feeder;
        this.indexer = indexer;
        this.hood = hood;
        this.flywheels = flywheels;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    private void handleStateTransitions() {
        switch (wantedSuperState) {
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
            case STOPPED:
            default:

        }
    }

    private boolean areSystemsReadyForShot() {
        // flywheels.flywheelsAtSetpoint() && hood.hoodAtSetpoint() && swerve.isAimed
        return false;
    }
    
}
