package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.spi.CurrencyNameProvider;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbWantedState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.Feeder.FeederWantedState;
import frc.robot.subsystems.indexer.Indexer;
// import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.intake.intakeWrist.IntakeWrist;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.vision6.Vision;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.HubShiftUtil.ShiftInfo;
import frc.robot.util.shootingutil.CalculateShot;
import frc.robot.util.shootingutil.ShotData;
import frc.robot.util.shootingutil.CalculateShot.AdjustedShot;
import frc.robot.util.Tracer;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.flywheels.Flywheels.FlywheelWantedState;
import frc.robot.util.AllianceFlipUtil;

public class Superstructure extends SubsystemBase {

    final CommandSwerveDrivetrain drivetrain;
    final Feeder feeder;
    final Indexer indexer;
    final IntakeRollers intakeRollers;
    final IntakeWrist intakeWrist;
    final Flywheels flywheels;
    final Hood hood;
    // final Climb climb;
    final LEDs leds;
    final Vision vision;

    public enum WantedSuperState {
        OUTAKE,
        // PREPARE_HUB_SHOT,
        // PREPARE_HUB_SHOT_AND_AGITATE,
        // PREPARE_ALLIANCE_ZONE_SHOT,
        // PREPARE_ALLIANCE_ZONE_SHOT_AND_AGITATE,
        SHOOT_HUB,
        SHOOT_HUB_AND_AGITATE,
        SOTM_HUB_AUTO,
        SHOOT_HUB_MANUAL,
        SHOOT_ALLIANCE_ZONE,
        SHOOT_ALLIANCE_ZONE_AND_AGITATE,
        IDLE,
        IDLE_AUTO,
        CLEAN,
        LIFT_INTAKE_AUTO,
        STOP,
        TESTING,
        // STOW
    }

    public enum CurrentSuperState {
        OUTAKING,
        UNJAMMING,
        PREPARING_HUB_SHOT,
        PREPARING_HUB_SHOT_AND_AGITATING,
        PREPARING_ALLIANCE_ZONE_SHOT,
        PREPARING_ALLIANCE_ZONE_SHOT_AND_AGITATING,
        PREPARING_SOTM_HUB_AUTO,
        SHOOTING_HUB,
        SHOOTING_HUB_AND_AGITATING,
        SHOOTING_ALLIANCE_ZONE,
        SHOOTING_ALLIANCE_ZONE_AND_AGITATING,
        SOTMING_HUB_AUTO,
        IDLING,
        IDLING_AUTO,
        CLEANING,
        LIFTING_INTAKE_AUTO,
        STOPPED,
        TESTING,
        // STOWING
    }

    private WantedSuperState wantedSuperState = WantedSuperState.STOP;
    private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
    private AdjustedShot adjustedShot = new AdjustedShot(Rotation2d.kZero, 0, 0, 0);

    public Superstructure(
            CommandSwerveDrivetrain drivetrain,
            Feeder feeder,
            Indexer indexer,
            IntakeRollers intakeRollers,
            IntakeWrist intakeWrist,
            Flywheels flywheels,
            Hood hood,
            Vision vision,
            // Climb climb,
            LEDs leds) {
        this.drivetrain = drivetrain;
        this.feeder = feeder;
        this.indexer = indexer;
        this.intakeRollers = intakeRollers;
        this.intakeWrist = intakeWrist;
        this.flywheels = flywheels;
        this.hood = hood;
        this.vision = vision;
        // this.climb = climb;
        this.leds = leds;
    }

    @Override
    public void periodic() {
        Tracer.startTrace("SuperstructurePeriodic");

        // Tracer.traceFunc("Climb Periodic", climb::updateInputs);
        Tracer.traceFunc("Drivetrain Periodic", drivetrain::updateInputs);
        Tracer.traceFunc("FeederPeriodic", feeder::updateInputs);
        Tracer.traceFunc("IndexerPeriodic", indexer::updateInputs);
        Tracer.traceFunc("IntakeRollers Periodic", intakeRollers::updateInputs);
        Tracer.traceFunc("IntakeWrist Periodic", intakeWrist::updateInputs);
        Tracer.traceFunc("FlywheelsPeriodic", flywheels::updateInputs);
        Tracer.traceFunc("HoodPeriodic", hood::updateInputs);
        Tracer.traceFunc("LEDsPeriodic", leds::updateInputs);
        Tracer.traceFunc("VisionPeriodic", vision::updateInputs);

        ShiftInfo shiftInfo = HubShiftUtil.getShiftInfo();
        DogLog.log("HubShift", shiftInfo.currentShift());
        DogLog.log("HubShift/ElapsedTime", shiftInfo.elapsedTime());
        DogLog.log("HubShift/RemainingTime", shiftInfo.remainingTime());
        DogLog.log("HubShift/Active", shiftInfo.active());

        Tracer.traceFunc("HandleStateTransitions", this::handleStateTransitions);
        Tracer.traceFunc("ApplyStates", this::applyStates);
        // This method will be called once per scheduler run
        DogLog.log("Superstructure/WantedSuperState", wantedSuperState);
        DogLog.log("Superstructure/CurrentSuperState", currentSuperState);
        DogLog.log("Superstructure/ReadyToShootHub", areSystemsReadyForHubShot());
        DogLog.log("Superstructure/ReadyToShootAllianceZone", areSystemsReadyForAllianceZoneShot());

        DogLog.log("Superstructure/isHoodUnsafe", isHoodUnsafe());
        DogLog.log("Superstructure/isIntakeUnsafe", isIntakeUnsafe());

        Tracer.endTrace();
    }

    private void handleStateTransitions() {
        switch (wantedSuperState) {
            case OUTAKE:
                currentSuperState = CurrentSuperState.OUTAKING;
                break;
            // case PREPARE_HUB_SHOT:
            // this.adjustedShot =
            // CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
            // drivetrain.getFieldRelativeChassisSpeeds(),
            // drivetrain.getFieldRelativeAccelerations());
            // currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT;
            // break;
            // case PREPARE_HUB_SHOT_AND_AGITATE:
            // this.adjustedShot =
            // CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
            // drivetrain.getFieldRelativeChassisSpeeds(),
            // drivetrain.getFieldRelativeAccelerations());
            // currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT_AND_AGITATING;
            // break;
            case SHOOT_HUB:
                this.adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
                        drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
                if (isFuelJammed()) {
                    currentSuperState = CurrentSuperState.UNJAMMING;
                } else if (areSystemsReadyForHubShot(this.adjustedShot.flightTime())) {
                    currentSuperState = CurrentSuperState.SHOOTING_HUB;
                } else {
                    currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT;
                }
                break;
            case SHOOT_HUB_AND_AGITATE:
                this.adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
                        drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
                if (isFuelJammed()) {
                    currentSuperState = CurrentSuperState.UNJAMMING;
                } else if (areSystemsReadyForHubShot(this.adjustedShot.flightTime())) {
                    currentSuperState = CurrentSuperState.SHOOTING_HUB_AND_AGITATING;
                } else {
                    currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT_AND_AGITATING;
                }
                break;
            case SOTM_HUB_AUTO:
                this.adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
                        drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
                if (isFuelJammed()) {
                    currentSuperState = CurrentSuperState.UNJAMMING;
                } else if (areSystemsReadyForHubShot(this.adjustedShot.flightTime())) {
                    currentSuperState = CurrentSuperState.SOTMING_HUB_AUTO;
                } else {
                    currentSuperState = CurrentSuperState.PREPARING_SOTM_HUB_AUTO;
                }
                break;
            case SHOOT_HUB_MANUAL:
                this.adjustedShot = CalculateShot.calculateManualShot();
                if (isFuelJammed()) {
                    currentSuperState = CurrentSuperState.UNJAMMING;
                } else if (areSystemsReadyForHubShot()) {
                    currentSuperState = CurrentSuperState.SHOOTING_HUB;
                } else {
                    currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT;
                }
                break;
            // case PREPARE_ALLIANCE_ZONE_SHOT:
            // this.adjustedShot =
            // CalculateShot.calculateAllianceZoneAdjustedShot(drivetrain.getPose(),
            // drivetrain.getFieldRelativeChassisSpeeds(),
            // drivetrain.getFieldRelativeAccelerations());
            // currentSuperState = CurrentSuperState.PREPARING_ALLIANCE_ZONE_SHOT;
            // break;
            // case PREPARE_ALLIANCE_ZONE_SHOT_AND_AGITATE:
            // this.adjustedShot =
            // CalculateShot.calculateAllianceZoneAdjustedShot(drivetrain.getPose(),
            // drivetrain.getFieldRelativeChassisSpeeds(),
            // drivetrain.getFieldRelativeAccelerations());
            // currentSuperState =
            // CurrentSuperState.PREPARING_ALLIANCE_ZONE_SHOT_AND_AGITATING;
            // break;
            case SHOOT_ALLIANCE_ZONE:
                this.adjustedShot = CalculateShot.calculateAllianceZoneAdjustedShot(drivetrain.getPose(),
                        drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
                if (isFuelJammed()) {
                    currentSuperState = CurrentSuperState.UNJAMMING;
                } else if (areSystemsReadyForAllianceZoneShot()) {
                    currentSuperState = CurrentSuperState.SHOOTING_ALLIANCE_ZONE;
                } else {
                    currentSuperState = CurrentSuperState.PREPARING_ALLIANCE_ZONE_SHOT;
                }
                break;
            case SHOOT_ALLIANCE_ZONE_AND_AGITATE:
                this.adjustedShot = CalculateShot.calculateAllianceZoneAdjustedShot(drivetrain.getPose(),
                        drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
                if (isFuelJammed()) {
                    currentSuperState = CurrentSuperState.UNJAMMING;
                } else if (areSystemsReadyForAllianceZoneShot()) {
                    currentSuperState = CurrentSuperState.SHOOTING_ALLIANCE_ZONE_AND_AGITATING;
                } else {
                    currentSuperState = CurrentSuperState.PREPARING_ALLIANCE_ZONE_SHOT_AND_AGITATING;
                }
                break;
            case IDLE:
                currentSuperState = CurrentSuperState.IDLING;
                break;
            case IDLE_AUTO:
                currentSuperState = CurrentSuperState.IDLING_AUTO;
                break;
            case CLEAN:
                currentSuperState = CurrentSuperState.CLEANING;
                break;
            case STOP:
                currentSuperState = CurrentSuperState.STOPPED;
                break;
            case TESTING:
                currentSuperState = CurrentSuperState.TESTING;
                break;
            // case STOW:
            // currentSuperState = CurrentSuperState.STOWING;
            // break;
            case LIFT_INTAKE_AUTO:
                currentSuperState = CurrentSuperState.LIFTING_INTAKE_AUTO;
                break;
            default:
                currentSuperState = CurrentSuperState.STOPPED;
                break;
        }
    }

    private void applyStates() {
        switch (currentSuperState) {
            case OUTAKING:
                outaking();
                break;
            case UNJAMMING:
                unjamming();
                break;
            case PREPARING_HUB_SHOT:
                preparingHubShot();
                break;
            case PREPARING_HUB_SHOT_AND_AGITATING:
                preparingHubShotAndAgitating();
                break;
            case PREPARING_SOTM_HUB_AUTO:
                preparingHubSOTMAuto();
                break;
            case SHOOTING_HUB:
                shootingHub();
                break;
            case SHOOTING_HUB_AND_AGITATING:
                shootingHubAndAgitating();
                break;
            case SOTMING_HUB_AUTO:
                SOTMHubAuto();
                break;
            case PREPARING_ALLIANCE_ZONE_SHOT:
                preparingAllianceZoneShot();
                break;
            case PREPARING_ALLIANCE_ZONE_SHOT_AND_AGITATING:
                preparingAllianceZoneShotAndAgitating();
                break;
            case SHOOTING_ALLIANCE_ZONE:
                shootingAllianceZone();
                break;
            case SHOOTING_ALLIANCE_ZONE_AND_AGITATING:
                shootingAllianceZoneAndAgitating();
                break;
            case IDLING:
                idling();
                break;
            case IDLING_AUTO:
                idlingAuto();
                break;
            case CLEANING:
                cleaning();
                break;
            case STOPPED:
                stop();
                break;
            case TESTING:
                testing();
                break;
            // case STOWING:
            // stowing();
            // break;
            case LIFTING_INTAKE_AUTO:
                liftingIntakeAuto();
                break;
            default:
                stop();
                break;
        }
    }

    public void outaking() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
        feeder.setWantedState(Feeder.FeederWantedState.OUTAKE);
        indexer.setWantedState(Indexer.IndexerWantedState.OUTAKE);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.OUTAKE);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.IDLE);
        hood.setWantedState(Hood.HoodWantedState.STOW);
        leds.setWantedState(LEDs.LEDsWantedState.IDLE);
    }

    public void unjamming() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK,
                this.adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.OUTAKE);
        indexer.setWantedState(Indexer.IndexerWantedState.OUTAKE);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                this.adjustedShot.shootSpeed());
        hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                this.adjustedShot.hoodAngle());
        leds.setWantedState(LEDs.LEDsWantedState.IDLE);
    }

    public void preparingHubShot() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK,
                this.adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        indexer.setWantedState(Indexer.IndexerWantedState.STOPPED);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                this.adjustedShot.shootSpeed());
        if (isHoodUnsafe()) {
            hood.setWantedState(Hood.HoodWantedState.STOW);
        } else {
            hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                    this.adjustedShot.hoodAngle());
        }
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_HUB_SHOT);
    }

    public void preparingHubShotAndAgitating() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK,
                this.adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        indexer.setWantedState(Indexer.IndexerWantedState.STOPPED);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                this.adjustedShot.shootSpeed());
        if (isHoodUnsafe()) {
            hood.setWantedState(Hood.HoodWantedState.STOW);
        } else {
            hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                    this.adjustedShot.hoodAngle());
        }
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_HUB_SHOT);
    }

    public void preparingHubSOTMAuto() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK_AND_FOLLOW_CHOREO_TRAJECTORY,
                this.adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        indexer.setWantedState(Indexer.IndexerWantedState.STOPPED);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                this.adjustedShot.shootSpeed());
        if (isHoodUnsafe()) {
            hood.setWantedState(Hood.HoodWantedState.STOW);
        } else {
            hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                    this.adjustedShot.hoodAngle());
        }
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_HUB_SHOT);
    }

    public void shootingHub() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK,
                this.adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                this.adjustedShot.shootSpeed());
        if (isHoodUnsafe()) {
            hood.setWantedState(Hood.HoodWantedState.STOW);
        } else {
            hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                    this.adjustedShot.hoodAngle());
        }
        leds.setWantedState(LEDs.LEDsWantedState.SHOOT_HUB);
    }

    public void shootingHubAndAgitating() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK,
                this.adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                this.adjustedShot.shootSpeed());
        if (isHoodUnsafe()) {
            hood.setWantedState(Hood.HoodWantedState.STOW);
        } else {
            hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                    this.adjustedShot.hoodAngle());
        }
        leds.setWantedState(LEDs.LEDsWantedState.SHOOT_HUB);
    }

    public void SOTMHubAuto() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK_AND_FOLLOW_CHOREO_TRAJECTORY,
                this.adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                this.adjustedShot.shootSpeed());
        if (isHoodUnsafe()) {
            hood.setWantedState(Hood.HoodWantedState.STOW);
        } else {
            hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                    this.adjustedShot.hoodAngle());
        }
        leds.setWantedState(LEDs.LEDsWantedState.SHOOT_HUB);
    }

    public void preparingAllianceZoneShot() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK, adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        indexer.setWantedState(Indexer.IndexerWantedState.STOPPED);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                adjustedShot.shootSpeed());
        if (isHoodUnsafe()) {
            hood.setWantedState(Hood.HoodWantedState.STOW);
        } else {
            hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                    this.adjustedShot.hoodAngle());
        }
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_ALLIANCE_ZONE_SHOT);
    }

    public void preparingAllianceZoneShotAndAgitating() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK, adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        indexer.setWantedState(Indexer.IndexerWantedState.STOPPED);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                adjustedShot.shootSpeed());
        if (isHoodUnsafe()) {
            hood.setWantedState(Hood.HoodWantedState.STOW);
        } else {
            hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                    this.adjustedShot.hoodAngle());
        }
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_ALLIANCE_ZONE_SHOT);
    }

    public void shootingAllianceZone() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK, adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                adjustedShot.shootSpeed());
        if (isHoodUnsafe()) {
            hood.setWantedState(Hood.HoodWantedState.STOW);
        } else {
            hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                    this.adjustedShot.hoodAngle());
        }
        leds.setWantedState(LEDs.LEDsWantedState.SHOOT_ALLIANCE_ZONE);
    }

    public void shootingAllianceZoneAndAgitating() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK, adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                adjustedShot.shootSpeed());
        if (isHoodUnsafe()) {
            hood.setWantedState(Hood.HoodWantedState.STOW);
        } else {
            hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                    this.adjustedShot.hoodAngle());
        }
        leds.setWantedState(LEDs.LEDsWantedState.SHOOT_ALLIANCE_ZONE);
    }

    public void idling() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
        feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
        indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        if (vision.hasTargets() && isIntakeUnsafe()) {
            intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.LIFT);
        } else {
            intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        }
        flywheels.setWantedState(Flywheels.FlywheelWantedState.IDLE);
        hood.setWantedState(Hood.HoodWantedState.STOW);
        leds.setWantedState(LEDs.LEDsWantedState.IDLE);
    }

    public void idlingAuto() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.CHOREO_TRAJECTORY);
        feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
        indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.IDLE);
        hood.setWantedState(Hood.HoodWantedState.STOW);
        leds.setWantedState(LEDs.LEDsWantedState.IDLE);
        // if (!isReadyToStowClimb() && !climb.atStowSetpoint()) {
        // climb.setWantedState(ClimbWantedState.EXTEND);
        // } else {
        // climb.setWantedState(ClimbWantedState.STOW);
        // }
    }

    public void liftingIntakeAuto() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.CHOREO_TRAJECTORY);
        feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
        indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.LIFT);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.IDLE);
        hood.setWantedState(Hood.HoodWantedState.STOW);
        leds.setWantedState(LEDs.LEDsWantedState.IDLE);
    }

    public void cleaning() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.STOPPED);
        feeder.setWantedState(Feeder.FeederWantedState.CLEAN);
        indexer.setWantedState(Indexer.IndexerWantedState.CLEAN);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.CLEAN);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.CLEAN);
        hood.setWantedState(Hood.HoodWantedState.STOW);
        leds.setWantedState(LEDs.LEDsWantedState.IDLE);
    }

    public void stop() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
        feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        indexer.setWantedState(Indexer.IndexerWantedState.STOPPED);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.STOP);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.STOP);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.STOPPED);
        hood.setWantedState(Hood.HoodWantedState.STOPPED);
        leds.setWantedState(LEDs.LEDsWantedState.STOP);
        // climb.setWantedState(ClimbWantedState.STOP);
    }

    public void testing() {
    // drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
    // if (areSystemsReadyForHubShot()) {
    // feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
    // indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
    // intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
    // } else {
    // feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
    // indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
    // intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
    // }
    // intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
    // flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS, 70);
    // hood.setWantedState(Hood.HoodWantedState.SET_POSITION, 0.01);
    }

    // public void stowing() {
    // drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
    // feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
    // indexer.setWantedState(Indexer.IndexerWantedState.STOPPED);
    // intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.STOP);
    // intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
    // flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS, 0);
    // hood.setWantedState(Hood.HoodWantedState.SET_POSITION, 0.1);
    // }

    private boolean areSystemsReadyForHubShot(double flightTime) {
        return flywheels.atSetpoint() && hood.atSetpoint() &&
                drivetrain.isAtTargetRotation()
                && HubShiftUtil.isHubPredictedActive(flightTime)
                && isWithinAllianceZone()
                ;
    }

    private boolean areSystemsReadyForHubShot() {
        return flywheels.atSetpoint() && hood.atSetpoint() &&
                drivetrain.isAtTargetRotation() 
                && isWithinAllianceZone()
                ;
    }

    public boolean isWithinAllianceZone() {
        if (AllianceFlipUtil.shouldFlip()) { // if red
            if (getPose().getX() > AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone)) {
                return true;
            } else {
                return false;
            }
        } else {
            if (getPose().getX() < AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone)) {
                return true;
            } else {
                return false;
            }
        }
    }

    private boolean areSystemsReadyForAllianceZoneShot() {
        return flywheels.atSetpoint() && hood.atSetpoint() &&
                drivetrain.isAtTargetRotation();
    }

    private boolean isFuelJammed() {
        return feeder.isFuelJammedFeeder();
    }

    private boolean isHoodUnsafe() {
        Translation2d pose = drivetrain.getPose().getTranslation();
        return FieldConstants.LeftTrench.trenchZone.contains(pose)
                || FieldConstants.RightTrench.trenchZone.contains(pose)
                || FieldConstants.LeftTrench.oppTrenchZone.contains(pose)
                || FieldConstants.RightTrench.oppTrenchZone.contains(pose);
    }

    private boolean isIntakeUnsafe() {
        Translation2d pose = drivetrain.getPose().getTranslation();
        return FieldConstants.LeftBump.bumpZone.contains(pose)
                || FieldConstants.RightBump.bumpZone.contains(pose)
                || FieldConstants.LeftBump.oppBumpZone.contains(pose)
                || FieldConstants.RightBump.oppBumpZone.contains(pose);
    }

    // private boolean isReadyToAdvanceToInnerTower(Pose2d outerTowerPose) {
    // return climb.atExtensionSetpoint() &&
    // drivetrain.isAtTargetPose(outerTowerPose);
    // }

    // private boolean isReadyToClimb(Pose2d innerTowerPose) {
    // return climb.atExtensionSetpoint() &&
    // drivetrain.isAtTargetPose(innerTowerPose);
    // }

    private boolean isReadyToStowClimb() {
        return !FieldConstants.Tower.towerZone.contains(drivetrain.getPose().getTranslation());
    }

    public Command zeroGyroCommand() {
        return this.runOnce(() -> drivetrain.zeroGyro());
    }

    public Command zeroPoseCommand() {
        return this.runOnce(() -> {
            drivetrain.zeroGyro();
            drivetrain.resetPose(new Pose2d(0, 0, new Rotation2d(0)));
        });
    }

    public Command setWantedSuperStateCommand(WantedSuperState wantedState) {
        return Commands.runOnce(() -> setWantedSuperState(wantedState));
    }

    public void setWantedSuperState(WantedSuperState state) {
        wantedSuperState = state;
    }

    public Pose2d getPose() {
        return drivetrain.getPose();
    }
}
