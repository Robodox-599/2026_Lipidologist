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
        // CLIMB,
        // DESCEND_CLIMB,
        IDLE,
        IDLE_AUTO,
        STOP,
        // TESTING,
        // STOW
    }

    public enum CurrentSuperState {
        OUTAKING,
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
        // ALIGN_OUTER_LEFT_TOWER,
        // ALIGN_INNER_LEFT_TOWER,
        // ALIGN_OUTER_RIGHT_TOWER,
        // ALIGN_INNER_RIGHT_TOWER,
        // CLIMB,
        // DESCEND_CLIMB,
        STOPPED,
        // TESTING,
        // STOWING
    }

    private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
    private CurrentSuperState currentSuperState = CurrentSuperState.IDLING;
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

        Tracer.endTrace();
    }

    private void handleStateTransitions() {
        switch (wantedSuperState) {
            case OUTAKE:
                currentSuperState = CurrentSuperState.OUTAKING;
                break;
            // case PREPARE_HUB_SHOT:
            //     this.adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
            //             drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
            //     currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT;
            //     break;
            // case PREPARE_HUB_SHOT_AND_AGITATE:  
            //     this.adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
            //             drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
            //     currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT_AND_AGITATING;
            //     break;
            case SHOOT_HUB:
                this.adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
                        drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
                if (areSystemsReadyForHubShot(this.adjustedShot.flightTime())) {
                    currentSuperState = CurrentSuperState.SHOOTING_HUB;
                } else {
                    currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT;
                }
                break;
            case SHOOT_HUB_AND_AGITATE:
                this.adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
                        drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
                if (areSystemsReadyForHubShot(this.adjustedShot.flightTime())) {
                    currentSuperState = CurrentSuperState.SHOOTING_HUB_AND_AGITATING;
                } else {
                    currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT_AND_AGITATING;
                }
                break;
            case SOTM_HUB_AUTO:
                this.adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
                        drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
                if (areSystemsReadyForHubShot()) {
                    currentSuperState = CurrentSuperState.SOTMING_HUB_AUTO;
                } else {
                    currentSuperState = CurrentSuperState.PREPARING_SOTM_HUB_AUTO;
                }
                break;
            case SHOOT_HUB_MANUAL:
                this.adjustedShot = CalculateShot.calculateManualShot();
                if (areSystemsReadyForHubShot()) {
                    currentSuperState = CurrentSuperState.SHOOTING_HUB;
                } else {
                    currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT;
                }
                break;
            // case PREPARE_ALLIANCE_ZONE_SHOT:
            //     this.adjustedShot = CalculateShot.calculateAllianceZoneAdjustedShot(drivetrain.getPose(),
            //             drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
            //     currentSuperState = CurrentSuperState.PREPARING_ALLIANCE_ZONE_SHOT;
            //     break;
            // case PREPARE_ALLIANCE_ZONE_SHOT_AND_AGITATE:
            //     this.adjustedShot = CalculateShot.calculateAllianceZoneAdjustedShot(drivetrain.getPose(),
            //             drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
            //     currentSuperState = CurrentSuperState.PREPARING_ALLIANCE_ZONE_SHOT_AND_AGITATING;
            //     break;
            case SHOOT_ALLIANCE_ZONE:
                this.adjustedShot = CalculateShot.calculateAllianceZoneAdjustedShot(drivetrain.getPose(),
                        drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
                if (areSystemsReadyForAllianceZoneShot()) {
                    currentSuperState = CurrentSuperState.SHOOTING_ALLIANCE_ZONE;
                } else {
                    currentSuperState = CurrentSuperState.PREPARING_ALLIANCE_ZONE_SHOT;
                }
                break;
            case SHOOT_ALLIANCE_ZONE_AND_AGITATE:
                this.adjustedShot = CalculateShot.calculateAllianceZoneAdjustedShot(drivetrain.getPose(),
                        drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
                if (areSystemsReadyForAllianceZoneShot()) {
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
            // case CLIMB:
            //     if (AllianceFlipUtil.shouldFlip()) { // if red alliance
            //         if (drivetrain.getPose().getY() >= AllianceFlipUtil.apply(FieldConstants.Tower.centerPoint)
            //                 .getY()) {
            //             if (isReadyToClimb(AllianceFlipUtil.apply(FieldConstants.Tower.rightInnerTowerPose))
            //                     || currentSuperState == CurrentSuperState.CLIMB) {
            //                 currentSuperState = CurrentSuperState.CLIMB;
            //             } else if (isReadyToAdvanceToInnerTower(AllianceFlipUtil.apply(FieldConstants.Tower.rightOuterTowerPose))
            //                     || currentSuperState == CurrentSuperState.ALIGN_INNER_RIGHT_TOWER) {
            //                 currentSuperState = CurrentSuperState.ALIGN_INNER_RIGHT_TOWER;
            //             } else {
            //                 currentSuperState = CurrentSuperState.ALIGN_OUTER_RIGHT_TOWER;
            //             }
            //         } else {
            //             if (isReadyToClimb(AllianceFlipUtil.apply(FieldConstants.Tower.leftInnerTowerPose))
            //                     || currentSuperState == CurrentSuperState.CLIMB) {
            //                 currentSuperState = CurrentSuperState.CLIMB;
            //             } else if (isReadyToAdvanceToInnerTower(AllianceFlipUtil.apply(FieldConstants.Tower.leftOuterTowerPose))
            //                     || currentSuperState == CurrentSuperState.ALIGN_INNER_LEFT_TOWER) {
            //                 currentSuperState = CurrentSuperState.ALIGN_INNER_LEFT_TOWER;
            //             } else {
            //                 currentSuperState = CurrentSuperState.ALIGN_OUTER_LEFT_TOWER;
            //             }
            //         }
            //     } else { // if blue alliance
            //         if (drivetrain.getPose().getY() >= FieldConstants.Tower.centerPoint.getY()) {
            //             if (isReadyToClimb(FieldConstants.Tower.leftInnerTowerPose)
            //                     || currentSuperState == CurrentSuperState.CLIMB) {
            //                 currentSuperState = CurrentSuperState.CLIMB;
            //             } else if (isReadyToAdvanceToInnerTower(FieldConstants.Tower.leftOuterTowerPose)
            //                     || currentSuperState == CurrentSuperState.ALIGN_INNER_LEFT_TOWER) {
            //                 currentSuperState = CurrentSuperState.ALIGN_INNER_LEFT_TOWER;
            //             } else {
            //                 currentSuperState = CurrentSuperState.ALIGN_OUTER_LEFT_TOWER;
            //             }
            //         } else {
            //             if (isReadyToClimb(FieldConstants.Tower.rightInnerTowerPose)
            //                     || currentSuperState == CurrentSuperState.CLIMB) {
            //                 currentSuperState = CurrentSuperState.CLIMB;
            //             } else if (isReadyToAdvanceToInnerTower(FieldConstants.Tower.rightOuterTowerPose)
            //                     || currentSuperState == CurrentSuperState.ALIGN_INNER_RIGHT_TOWER) {
            //                 currentSuperState = CurrentSuperState.ALIGN_INNER_RIGHT_TOWER;
            //             } else {
            //                 currentSuperState = CurrentSuperState.ALIGN_OUTER_RIGHT_TOWER;
            //             }
            //         }
            //     }
            //     break;
            // case DESCEND_CLIMB:
            // currentSuperState = CurrentSuperState.DESCEND_CLIMB;
            // break;
            case STOP:
                currentSuperState = CurrentSuperState.STOPPED;
                break;
            // case TESTING:
            //     currentSuperState = CurrentSuperState.TESTING;
            //     break;
            // case STOW:
            //     currentSuperState = CurrentSuperState.STOWING;
            //     break;
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
            // case ALIGN_OUTER_LEFT_TOWER:
            //     alignOuterLeftTower();
            //     break;
            // case ALIGN_INNER_LEFT_TOWER:
            //     alignInnerLeftTower();
            //     break;
            // case ALIGN_OUTER_RIGHT_TOWER:
            //     alignOuterRightTower();
            //     break;
            // case ALIGN_INNER_RIGHT_TOWER:
            //     alignInnerRightTower();
            //     break;
            // case CLIMB:
            //     climb();
            //     break;
            // case DESCEND_CLIMB:
            // descendClimb();
            // break;
            case IDLING:
                idling();
                break;
            case IDLING_AUTO:
                idlingAuto();
                break;
            case STOPPED:
                stop();
                break;
            // case TESTING:
            //     testing();
            //     break;
            // case STOWING:
            //     stowing();
            //     break;
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

    public void preparingHubShot() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK,
                this.adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
        indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                this.adjustedShot.shootSpeed());
        // if (isHoodUnsafe()) {
        //     hood.setWantedState(Hood.HoodWantedState.STOW);
        // } else {
            hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                    this.adjustedShot.hoodAngle());
        // }
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_HUB_SHOT);
        // if (!isReadyToStowClimb() && !climb.atStowSetpoint()) {
        // climb.setWantedState(ClimbWantedState.EXTEND);
        // } else {
        // climb.setWantedState(ClimbWantedState.STOW);
        // }
    }

    public void preparingHubShotAndAgitating() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK,
                this.adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
        indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.AGITATE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                this.adjustedShot.shootSpeed());
        // if (isHoodUnsafe()) {
        //     hood.setWantedState(Hood.HoodWantedState.STOW);
        // } else {
            hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                    this.adjustedShot.hoodAngle());
        // }
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_HUB_SHOT);
        // if (!isReadyToStowClimb() && !climb.atStowSetpoint()) {
        // climb.setWantedState(ClimbWantedState.EXTEND);
        // } else {
        // climb.setWantedState(ClimbWantedState.STOW);
        // }
    }

    public void preparingHubSOTMAuto() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK_AND_FOLLOW_CHOREO_TRAJECTORY,
                this.adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
        indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                this.adjustedShot.shootSpeed());
        // if (isHoodUnsafe()) {
        //     hood.setWantedState(Hood.HoodWantedState.STOW);
        // } else {
            hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                    this.adjustedShot.hoodAngle());
        // }
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_HUB_SHOT);
        // if (!isReadyToStowClimb() && !climb.atStowSetpoint()) {
        // climb.setWantedState(ClimbWantedState.EXTEND);
        // } else {
        // climb.setWantedState(ClimbWantedState.STOW);
        // }
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
        hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                this.adjustedShot.hoodAngle());
        leds.setWantedState(LEDs.LEDsWantedState.SHOOT_HUB);
        // if (!isReadyToStowClimb() && !climb.atStowSetpoint()) {
        // climb.setWantedState(ClimbWantedState.EXTEND);
        // } else {
        // climb.setWantedState(ClimbWantedState.STOW);
        // }
    }

    public void shootingHubAndAgitating() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK,
                this.adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.AGITATE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                this.adjustedShot.shootSpeed());
        hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                this.adjustedShot.hoodAngle());
        leds.setWantedState(LEDs.LEDsWantedState.SHOOT_HUB);
        // if (!isReadyToStowClimb() && !climb.atStowSetpoint()) {
        // climb.setWantedState(ClimbWantedState.EXTEND);
        // } else {
        // climb.setWantedState(ClimbWantedState.STOW);
        // }
    }

    public void SOTMHubAuto() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK_AND_FOLLOW_CHOREO_TRAJECTORY,
                this.adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                this.adjustedShot.shootSpeed());
        hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                this.adjustedShot.hoodAngle());
        leds.setWantedState(LEDs.LEDsWantedState.SHOOT_HUB);
        // if (!isReadyToStowClimb() && !climb.atStowSetpoint()) {
        // climb.setWantedState(ClimbWantedState.EXTEND);
        // } else {
        // climb.setWantedState(ClimbWantedState.STOW);
        // }
    }

    public void preparingAllianceZoneShot() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK, adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
        indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                adjustedShot.shootSpeed());
        // if (isHoodUnsafe()) {
        //     hood.setWantedState(Hood.HoodWantedState.STOW);
        // } else {
            hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                    this.adjustedShot.hoodAngle());
        // }
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_ALLIANCE_ZONE_SHOT);
        // if (!isReadyToStowClimb() && !climb.atStowSetpoint()) {
        // climb.setWantedState(ClimbWantedState.EXTEND);
        // } else {
        // climb.setWantedState(ClimbWantedState.STOW);
        // }
    }

    public void preparingAllianceZoneShotAndAgitating() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK, adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
        indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.AGITATE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                adjustedShot.shootSpeed());
        // if (isHoodUnsafe()) {
        //     hood.setWantedState(Hood.HoodWantedState.STOW);
        // } else {
            hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                    this.adjustedShot.hoodAngle());
        // }
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_ALLIANCE_ZONE_SHOT);
        // if (!isReadyToStowClimb() && !climb.atStowSetpoint()) {
        // climb.setWantedState(ClimbWantedState.EXTEND);
        // } else {
        // climb.setWantedState(ClimbWantedState.STOW);
        // }
    }

    public void shootingAllianceZone() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK, adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                adjustedShot.shootSpeed());
        hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                adjustedShot.hoodAngle());
        leds.setWantedState(LEDs.LEDsWantedState.SHOOT_ALLIANCE_ZONE);
        // if (!isReadyToStowClimb() && !climb.atStowSetpoint()) {
        // climb.setWantedState(ClimbWantedState.EXTEND);
        // } else {
        // climb.setWantedState(ClimbWantedState.STOW);
        // }
    }

    public void shootingAllianceZoneAndAgitating() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK, adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.AGITATE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
                adjustedShot.shootSpeed());
        hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
                adjustedShot.hoodAngle());
        leds.setWantedState(LEDs.LEDsWantedState.SHOOT_ALLIANCE_ZONE);
        // if (!isReadyToStowClimb() && !climb.atStowSetpoint()) {
        // climb.setWantedState(ClimbWantedState.EXTEND);
        // } else {
        // climb.setWantedState(ClimbWantedState.STOW);
        // }
    }


    public void idling() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
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

    // public void alignOuterLeftTower() {
    //     drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.DRIVE_TO_POINT,
    //             AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.apply(FieldConstants.Tower.leftOuterTowerPose)
    //                     : FieldConstants.Tower.leftOuterTowerPose);
    //     feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
    //     indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
    //     intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
    //     intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
    //     flywheels.setWantedState(Flywheels.FlywheelWantedState.IDLE);
    //     if (isHoodUnsafe()) {
    //         hood.setWantedState(Hood.HoodWantedState.STOW);
    //     } else {
    //         hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
    //                 this.adjustedShot.hoodAngle());
    //     }
    //     leds.setWantedState(LEDs.LEDsWantedState.IDLE);
    //     // climb.setWantedState(ClimbWantedState.EXTEND);
    // }

    // public void alignInnerLeftTower() {
    //     drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.DRIVE_TO_POINT,
    //             AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.apply(FieldConstants.Tower.leftInnerTowerPose)
    //                     : FieldConstants.Tower.leftInnerTowerPose);
    //     feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
    //     indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
    //     intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
    //     intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
    //     flywheels.setWantedState(Flywheels.FlywheelWantedState.IDLE);
    //     if (isHoodUnsafe()) {
    //         hood.setWantedState(Hood.HoodWantedState.STOW);
    //     } else {
    //         hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
    //                 this.adjustedShot.hoodAngle());
    //     }
    //     leds.setWantedState(LEDs.LEDsWantedState.IDLE);
    //     // climb.setWantedState(ClimbWantedState.EXTEND);
    // }

    // public void alignOuterRightTower() {
    //     drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.DRIVE_TO_POINT,
    //             AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.apply(FieldConstants.Tower.rightOuterTowerPose)
    //                     : FieldConstants.Tower.rightOuterTowerPose);
    //     feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
    //     indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
    //     intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
    //     intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
    //     flywheels.setWantedState(Flywheels.FlywheelWantedState.IDLE);
    //     if (isHoodUnsafe()) {
    //         hood.setWantedState(Hood.HoodWantedState.STOW);
    //     } else {
    //         hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
    //                 this.adjustedShot.hoodAngle());
    //     }
    //     leds.setWantedState(LEDs.LEDsWantedState.IDLE);
    //     // climb.setWantedState(ClimbWantedState.EXTEND);
    // }

    // public void alignInnerRightTower() {
    //     drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.DRIVE_TO_POINT,
    //             AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.apply(FieldConstants.Tower.rightInnerTowerPose)
    //                     : FieldConstants.Tower.rightInnerTowerPose);
    //     feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
    //     indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
    //     intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
    //     intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
    //     flywheels.setWantedState(Flywheels.FlywheelWantedState.IDLE);
    //     if (isHoodUnsafe()) {
    //         hood.setWantedState(Hood.HoodWantedState.STOW);
    //     } else {
    //         hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
    //                 this.adjustedShot.hoodAngle());
    //     }
    //     leds.setWantedState(LEDs.LEDsWantedState.IDLE);
    //     // climb.setWantedState(ClimbWantedState.EXTEND);
    // }

    // public void climb() {
    //     drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.STOPPED);
    //     feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
    //     indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
    //     intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
    //     intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
    //     flywheels.setWantedState(Flywheels.FlywheelWantedState.IDLE);
    //     if (isHoodUnsafe()) {
    //         hood.setWantedState(Hood.HoodWantedState.STOW);
    //     } else {
    //         hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
    //                 this.adjustedShot.hoodAngle());
    //     }
    //     leds.setWantedState(LEDs.LEDsWantedState.IDLE);
    //     // climb.setWantedState(ClimbWantedState.RETRACT);
    // }

    // public void descendClimb() {
    // drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
    // feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
    // indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
    // intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
    // intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
    // flywheels.setWantedState(Flywheels.FlywheelWantedState.IDLE);
    // if (isHoodUnsafe()) {
    // hood.setWantedState(Hood.HoodWantedState.STOW);
    // } else {
    // hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
    // this.adjustedShot.hoodAngle());
    // }
    // leds.setWantedState(LEDs.LEDsWantedState.IDLE);
    // climb.setWantedState(ClimbWantedState.EXTEND);
    // }

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

    // public void testing() {
    //     drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
    //     if (areSystemsReadyForHubShot()) {
    //         feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
    //         indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
    //         intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
    //     } else {
    //         feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
    //         indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
    //         intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
    //     }
    //     intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
    //     flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS, 70);
    //     hood.setWantedState(Hood.HoodWantedState.SET_POSITION, 0.01);
    // }

    // public void stowing() {
    //     drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
    //     feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
    //     indexer.setWantedState(Indexer.IndexerWantedState.STOPPED);
    //     intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.STOP);
    //     intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
    //     flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS, 0);
    //     hood.setWantedState(Hood.HoodWantedState.SET_POSITION, 0.1);
    // }

    private boolean areSystemsReadyForHubShot(double flightTime) {
        return flywheels.atSetpoint() && hood.atSetpoint() &&
                drivetrain.isAtTargetRotation()
                && HubShiftUtil.isHubPredictedActive(flightTime);
    }

    private boolean areSystemsReadyForHubShot() {
        return flywheels.atSetpoint() && hood.atSetpoint() &&
                drivetrain.isAtTargetRotation();
    }

    private boolean areSystemsReadyForAllianceZoneShot() {
        return flywheels.atSetpoint() && hood.atSetpoint() &&
                drivetrain.isAtTargetRotation();
    }

    private boolean isHoodUnsafe() {
        return FieldConstants.LeftTrench.trenchZone.contains(drivetrain.getPose().getTranslation())
                || FieldConstants.RightTrench.trenchZone.contains(drivetrain.getPose().getTranslation())
                || FieldConstants.LeftTrench.oppTrenchZone.contains(drivetrain.getPose().getTranslation())
                || FieldConstants.RightTrench.oppTrenchZone.contains(drivetrain.getPose().getTranslation());
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
        return this.runOnce(() -> setWantedSuperState(wantedState));
    }

    public void setWantedSuperState(WantedSuperState state) {
        wantedSuperState = state;
    }

    public Pose2d getPose() {
        return drivetrain.getPose();
    }
}
