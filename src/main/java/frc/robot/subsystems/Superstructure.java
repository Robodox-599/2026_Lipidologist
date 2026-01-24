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
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.Feeder.WantedState;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.intake.intakeWrist.IntakeWrist;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.CalculateShot;
import frc.robot.util.GetShotData;
import frc.robot.util.CalculateShot.AdjustedShot;
import frc.robot.subsystems.shooter.flywheels.Flywheels;

public class Superstructure extends SubsystemBase {

    final Climb climb;
    final CommandSwerveDrivetrain drivetrain;
    final Feeder feeder;
    final Indexer indexer;
    final IntakeRollers intakeRollers;
    final IntakeWrist intakeWrist;
    final Flywheels flywheels;
    final Hood hood;
    final Vision vision;
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
            Climb climb,
            CommandSwerveDrivetrain drivetrain,
            Feeder feeder,
            Indexer indexer,
            IntakeRollers intakeRollers,
            IntakeWrist intakeWrist,
            Flywheels flywheels,
            Hood hood,
            Vision vision) {
        this.climb = climb;
        this.drivetrain = drivetrain;
        this.feeder = feeder;
        this.indexer = indexer;
        this.intakeRollers = intakeRollers;
        this.intakeWrist = intakeWrist;
        this.flywheels = flywheels;
        this.hood = hood;
        this.vision = vision;
    }

    @Override
    public void periodic() {
        climb.updateInputs();
        drivetrain.updateInputs();
        feeder.updateInputs();
        indexer.updateInputs();
        intakeRollers.updateInputs();
        intakeWrist.updateInputs();
        flywheels.updateInputs();
        hood.updateInputs();
        vision.updateInputs();

        handleStateTransitions();
        applyStates();
        // This method will be called once per scheduler run
        DogLog.log("Superstructure/WantedSuperState", wantedSuperState);
        DogLog.log("Superstructure/CurrentSuperState", currentSuperState);
    }

    private void handleStateTransitions() {
        switch (wantedSuperState) {
            default:
            case PREPARE_HUB_SHOT:
                currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT;
                break;
            case SHOOT_HUB:
                if (areSystemsReadyForShot()) {
                    currentSuperState = CurrentSuperState.SHOOTING_HUB;
                } else {
                    currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT;
                }
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

    public void preparingHubShot() {
        AdjustedShot adjustedShot = CalculateShot.calculateAdjustedShot(drivetrain.getPose(),
                drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());

        drivetrain.setTargetRotation(adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.WantedState.STOPPED);
        indexer.setWantedState(Indexer.WantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.WantedState.INTAKING_FUEL);
        intakeWrist.setWantedState(IntakeWrist.WantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.WantedState.SET_RPM, adjustedShot.shootSpeed());
        hood.setWantedState(Hood.WantedState.SET_POSITION, adjustedShot.hoodAngle());
    }

    public void shootingHub() {
        AdjustedShot adjustedShot = CalculateShot.calculateAdjustedShot(drivetrain.getPose(),
                drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());

        drivetrain.setTargetRotation(adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.WantedState.FEED_FUEL);
        indexer.setWantedState(Indexer.WantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.WantedState.INTAKING_FUEL);
        intakeWrist.setWantedState(IntakeWrist.WantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.WantedState.SET_RPM, adjustedShot.shootSpeed());
        hood.setWantedState(Hood.WantedState.SET_POSITION, adjustedShot.hoodAngle());
    }

    public void idling() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
        feeder.setWantedState(Feeder.WantedState.STOPPED);
        indexer.setWantedState(Indexer.WantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.WantedState.INTAKING_FUEL);
        intakeWrist.setWantedState(IntakeWrist.WantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.WantedState.SET_RPM);
        hood.setWantedState(Hood.WantedState.SET_POSITION);
    }

    public void stop() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
        feeder.setWantedState(Feeder.WantedState.STOPPED);
        indexer.setWantedState(Indexer.WantedState.STOPPED);
        intakeRollers.setWantedState(IntakeRollers.WantedState.STOPPED);
        intakeWrist.setWantedState(IntakeWrist.WantedState.STOPPED);
        flywheels.setWantedState(Flywheels.WantedState.STOPPED);
        hood.setWantedState(Hood.WantedState.STOPPED);
    }

    private boolean areSystemsReadyForShot() {
        return flywheels.atSetpoint() && hood.atSetpoint() && drivetrain.isAtTargetRotation();
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
