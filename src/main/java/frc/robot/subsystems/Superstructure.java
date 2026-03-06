package frc.robot.subsystems;

import dev.doglog.DogLog;
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
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.Feeder.FeederWantedState;
import frc.robot.subsystems.indexer.Indexer;
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
    // final Feeder feeder;
    final Indexer indexer;
    final IntakeRollers intakeRollers;
    final IntakeWrist intakeWrist;
    // final Flywheels flywheels;
    // final Hood hood;
    // final Climb climb;
    final LEDs leds;
    // final Vision vision;

    public boolean firstDeploy = false;

    public enum WantedSuperState {
        PREPARE_HUB_SHOT,
        PREPARE_HUB_SHOT_ALIGNED_TROUGH,
        PREPARE_ALLIANCE_ZONE_SHOT,
        PREPARE_ALLIANCE_ZONE_SHOT_ALIGNED_TROUGH,
        SHOOT_HUB,
        SHOOT_ALLIANCE_ZONE,
        EXTEND_CLIMB,
        EXTEND_CLIMB_WHILE_PREPARING_HUB_SHOT,
        EXTEND_CLIMB_WHILE_SHOOTING_HUB,
        RETRACT_CLIMB_WHILE_SHOOTING,
        RETRACT_CLIMB,
        IDLE,
        STOP,
        TESTING,
        STOW
    }

    public enum CurrentSuperState {
        PREPARING_HUB_SHOT,
        PREPARING_HUB_SHOT_ALIGNED_TROUGH,
        PREPARING_ALLIANCE_ZONE_SHOT,
        SHOOTING_HUB,
        SHOOTING_ALLIANCE_ZONE,
        IDLING,
        EXTENDING_CLIMB,
        EXTENDING_CLIMB_WHILE_PREPARING_HUB_SHOT,
        EXTENDING_CLIMB_WHILE_SHOOTING_HUB,
        RETRACTING_CLIMB_WHILE_SHOOTING,
        RETRACTING_CLIMB,
        STOPPED,
        TESTING,
        STOWING
    }

    private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
    private CurrentSuperState currentSuperState = CurrentSuperState.IDLING;
    private AdjustedShot adjustedShot = new AdjustedShot(new Rotation2d(), 0, 0, 0);

    public Superstructure(
            CommandSwerveDrivetrain drivetrain,
            // Feeder feeder,
            Indexer indexer,
            IntakeRollers intakeRollers,
            IntakeWrist intakeWrist,
            // Flywheels flywheels,
            // Hood hood,
            // Vision vision,
            // Climb climb,
            LEDs leds) {
        this.drivetrain = drivetrain;
        // this.feeder = feeder;
        this.indexer = indexer;
        this.intakeRollers = intakeRollers;
        this.intakeWrist = intakeWrist;
        // this.flywheels = flywheels;
        // this.hood = hood;
        // this.vision = vision;
        // this.climb = climb;
        this.leds = leds;
    }

    @Override
    public void periodic() {
        Tracer.startTrace("SuperstructurePeriodic");

        // Tracer.traceFunc("Climb Periodic", climb::updateInputs);
        Tracer.traceFunc("Drivetrain Periodic", drivetrain::updateInputs);
        // Tracer.traceFunc("FeederPeriodic", feeder::updateInputs);
        Tracer.traceFunc("IndexerPeriodic", indexer::updateInputs);
        Tracer.traceFunc("IntakeRollers Periodic", intakeRollers::updateInputs);
        Tracer.traceFunc("IntakeWrist Periodic", intakeWrist::updateInputs);
        // Tracer.traceFunc("FlywheelsPeriodic", flywheels::updateInputs);
        // Tracer.traceFunc("HoodPeriodic", hood::updateInputs);
        Tracer.traceFunc("LEDsPeriodic", leds::updateInputs);
        // Tracer.traceFunc("VisionPeriodic", vision::updateInputs);

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

        Tracer.endTrace();
    }

    private void handleStateTransitions() {
        switch (wantedSuperState) {
            default:
            case PREPARE_HUB_SHOT:
                this.adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
                        drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
                currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT;
                break;
            case SHOOT_HUB:
                this.adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
                        drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
                // if (areSystemsReadyForHubShot(this.adjustedShot.flightTime())) {
                // currentSuperState = CurrentSuperState.SHOOTING_HUB;
                // } else {
                // currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT;
                // }
                // if (areSystemsReadyForAllianceZoneShot()) {
                currentSuperState = CurrentSuperState.SHOOTING_HUB;
                // } else {
                // currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT;
                // }
                break;
            case PREPARE_ALLIANCE_ZONE_SHOT:
                this.adjustedShot = CalculateShot.calculateAllianceZoneAdjustedShot(drivetrain.getPose(),
                        drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
                currentSuperState = CurrentSuperState.PREPARING_ALLIANCE_ZONE_SHOT;
                break;
            case SHOOT_ALLIANCE_ZONE:
                this.adjustedShot = CalculateShot.calculateAllianceZoneAdjustedShot(drivetrain.getPose(),
                        drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
                // if (areSystemsReadyForAllianceZoneShot()) {
                currentSuperState = CurrentSuperState.SHOOTING_ALLIANCE_ZONE;
                // } else {
                // currentSuperState = CurrentSuperState.PREPARING_ALLIANCE_ZONE_SHOT;
                // }
                break;
            case EXTEND_CLIMB:
                currentSuperState = CurrentSuperState.EXTENDING_CLIMB;
                break;
            case EXTEND_CLIMB_WHILE_PREPARING_HUB_SHOT:
                this.adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
                        drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
                currentSuperState = CurrentSuperState.EXTENDING_CLIMB_WHILE_PREPARING_HUB_SHOT;
                break;
            case EXTEND_CLIMB_WHILE_SHOOTING_HUB:
                this.adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
                        drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());
                // if (areSystemsReadyForHubZoneShot()) {
                // currentSuperState = CurrentSuperState.EXTENDING_CLIMB_WHILE_SHOOTING_HUB;
                // } else {
                // currentSuperState =
                // CurrentSuperState.EXTENDING_CLIMB_WHILE_PREPARING_HUB_SHOT;
                // }
                break;
            case RETRACT_CLIMB:
                // if (isClimbReady()) {
                // currentSuperState = CurrentSuperState.RETRACTING_CLIMB;
                // } else {
                // currentSuperState = CurrentSuperState.EXTENDING_CLIMB;
                // }
                break;
            case RETRACT_CLIMB_WHILE_SHOOTING:
                // if (isClimbReady()) {
                // currentSuperState = CurrentSuperState.RETRACTING_CLIMB_WHILE_SHOOTING;
                // } else {
                // currentSuperState = CurrentSuperState.EXTENDING_CLIMB_WHILE_SHOOTING_HUB;
                // }
                break;
            case IDLE:
                currentSuperState = CurrentSuperState.IDLING;
                break;
            case STOP:
                currentSuperState = CurrentSuperState.STOPPED;
                break;
            case TESTING:
                currentSuperState = CurrentSuperState.TESTING;
                break;
            case STOW:
                currentSuperState = CurrentSuperState.STOWING;
                break;
        }
    }

    private void applyStates() {
        switch (currentSuperState) {
            default:
            case PREPARING_HUB_SHOT:
                preparingHubShot();
                break;
            case SHOOTING_HUB:
                shootingHub();
                break;
            case PREPARING_ALLIANCE_ZONE_SHOT:
                preparingAllianceZoneShot();
                break;
            case SHOOTING_ALLIANCE_ZONE:
                shootingAllianceZone();
                break;
            case EXTENDING_CLIMB:
                extendingClimb();
                break;
            case EXTENDING_CLIMB_WHILE_PREPARING_HUB_SHOT:
                extendingClimbWhilePreparingHubShot();
                break;
            case EXTENDING_CLIMB_WHILE_SHOOTING_HUB:
                extendingClimbWhileShootingHub();
                break;
            case RETRACTING_CLIMB:
                retractingClimb();
                break;
            case RETRACTING_CLIMB_WHILE_SHOOTING:
                retractingClimbWhileShooting();
            case IDLING:
                idling();
                break;
            case STOPPED:
                stop();
                break;
            case TESTING:
                testing();
                break;
            case STOWING:
                stowing();
        }
    }

    public void preparingHubShot() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK,
                this.adjustedShot.targetRotation());
        // feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
        indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
        // this.adjustedShot.shootSpeed());
        // if (isHoodUnsafe()) {
        // hood.setWantedState(Hood.HoodWantedState.STOW);
        // } else {
        // hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
        // this.adjustedShot.hoodAngle());
        // }
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_HUB_SHOT);
    }

    public void shootingHub() {
        if (DriverStation.isAutonomous()) {
            drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK_AND_FOLLOW_CHOREO_TRAJECTORY,
                    this.adjustedShot.targetRotation());
        } else {
            drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK,
                    this.adjustedShot.targetRotation());
        }
        // feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
        // this.adjustedShot.shootSpeed());
        // hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
        // this.adjustedShot.hoodAngle());
        leds.setWantedState(LEDs.LEDsWantedState.SHOOT_HUB);
    }

    public void preparingAllianceZoneShot() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK, adjustedShot.targetRotation());
        // feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
        indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
        // adjustedShot.shootSpeed());
        // if (isHoodUnsafe()) {
        // hood.setWantedState(Hood.HoodWantedState.STOW);
        // } else {
        // hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
        // this.adjustedShot.hoodAngle());
        // }
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_ALLIANCE_ZONE_SHOT);
    }

    public void shootingAllianceZone() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK, adjustedShot.targetRotation());
        // feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
        // adjustedShot.shootSpeed());
        // hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
        // adjustedShot.hoodAngle());
        leds.setWantedState(LEDs.LEDsWantedState.SHOOT_ALLIANCE_ZONE);
    }

    public void idling() {
        if (DriverStation.isAutonomous()) {
            drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.CHOREO_TRAJECTORY);
        } else {
            drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
        }
        // feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
        indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.IDLE);
        // if (isHoodUnsafe()) {
        // hood.setWantedState(Hood.HoodWantedState.STOW);
        // } else {
        // hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
        // this.adjustedShot.hoodAngle());
        // }
        leds.setWantedState(LEDs.LEDsWantedState.IDLE);
    }

    public void extendingClimb() {
        if (DriverStation.isAutonomous()) {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK_AND_FOLLOW_CHOREO_TRAJECTORY,
        this.adjustedShot.targetRotation());
        } else {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.DRIVE_TO_POINT,
        getNearestTowerApproach());
        }
        // feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
        indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.STOW);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.IDLE);
        // hood.setWantedState(Hood.HoodWantedState.STOW);
        // climb.setWantedState(Climb.ClimbWantedState.EXTENDING);
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_CLIMB);
    }

    public void extendingClimbWhilePreparingHubShot() {
        if (DriverStation.isAutonomous()) {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK_AND_FOLLOW_CHOREO_TRAJECTORY,
        this.adjustedShot.targetRotation());
        } else {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.DRIVE_TO_POINT,
        getNearestTowerApproach());
        }
        // feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
        indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
        // this.adjustedShot.shootSpeed());
        // if (isHoodUnsafe()) {
        // hood.setWantedState(Hood.HoodWantedState.STOW);
        // } else {
        // hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
        // this.adjustedShot.hoodAngle());
        // }
        // climb.setWantedState(Climb.ClimbWantedState.EXTENDING);
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_CLIMB);
    }

    public void extendingClimbWhileShootingHub() {
        if (DriverStation.isAutonomous()) {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK_AND_FOLLOW_CHOREO_TRAJECTORY,
        this.adjustedShot.targetRotation());
        } else {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.DRIVE_TO_POINT,
        getNearestTowerApproach());
        }
        // feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.STOW);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
        // this.adjustedShot.shootSpeed());
        // hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
        // this.adjustedShot.hoodAngle());
        // climb.setWantedState(Climb.ClimbWantedState.EXTENDING);
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_CLIMB);
    }

    public void retractingClimbWhileShooting() {
        // if (DriverStation.isAutonomous()) {
        // drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK_AND_FOLLOW_CHOREO_TRAJECTORY,
        // this.adjustedShot.targetRotation());
        // } else {
        // drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.DRIVE_TO_POINT,
        // new Pose);
        // }
        // feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.STOW);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS,
        // this.adjustedShot.shootSpeed());
        // hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
        // this.adjustedShot.hoodAngle());
        // climb.setWantedState(Climb.ClimbWantedState.RETRACTING);
        leds.setWantedState(LEDs.LEDsWantedState.CLIMB);
    }

    public void retractingClimb() {
        // if (DriverStation.isAutonomous()) {
        // drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK_AND_FOLLOW_CHOREO_TRAJECTORY,
        // this.adjustedShot.targetRotation());
        // } else {
        // drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.DRIVE_TO_POINT,
        // new Pose);
        // }
        // feeder.setWantedState(Feeder.FeederWantedState.REVERSE);
        indexer.setWantedState(Indexer.IndexerWantedState.REVERSE);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.STOW);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.IDLE);
        // hood.setWantedState(Hood.HoodWantedState.STOW);
        // climb.setWantedState(Climb.ClimbWantedState.RETRACTING);
        leds.setWantedState(LEDs.LEDsWantedState.CLIMB);
    }

    public void stop() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
        // feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        indexer.setWantedState(Indexer.IndexerWantedState.STOPPED);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.STOP);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.STOP);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.STOPPED);
        // hood.setWantedState(Hood.HoodWantedState.STOPPED);
        leds.setWantedState(LEDs.LEDsWantedState.STOP);
    }

    public void testing() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
        // feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS, 95);
        // hood.setWantedState(Hood.HoodWantedState.SET_POSITION, 0.1);
    }

    public void stowing() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
        // feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        indexer.setWantedState(Indexer.IndexerWantedState.STOPPED);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.STOP);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS, 0);
        // hood.setWantedState(Hood.HoodWantedState.SET_POSITION, 0.1);
    }

    // private boolean areSystemsReadyForHubShot(double flightTime) {
    // return flywheels.atSetpoint() && hood.atSetpoint() &&
    // drivetrain.isAtTargetRotation()
    // && HubShiftUtil.isHubPredictedActive(flightTime);
    // }

    // private boolean areSystemsReadyForAllianceZoneShot() {
    // return flywheels.atSetpoint() && hood.atSetpoint() &&
    // drivetrain.isAtTargetRotation();
    // }

    private boolean isHoodUnsafe() {
        return FieldConstants.LeftTrench.trenchZone.contains(drivetrain.getPose().getTranslation())
                || FieldConstants.RightTrench.trenchZone.contains(drivetrain.getPose().getTranslation())
                || FieldConstants.LeftTrench.oppTrenchZone.contains(drivetrain.getPose().getTranslation())
                || FieldConstants.RightTrench.oppTrenchZone.contains(drivetrain.getPose().getTranslation());
    }

    // private boolean isClimbReady() {
    // return climb.atSetpoint() && drivetrain.isAtTargetRotation();
    // }

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

    public Pose2d getNearestTowerApproach() {
        Pose2d targetPose;

        if (AllianceFlipUtil.shouldFlip()) {
            if (drivetrain.getPose().getTranslation().getDistance(FieldConstants.Tower.oppLeftUpright) > drivetrain
                    .getPose().getTranslation().getDistance(FieldConstants.Tower.oppRightUpright)) {
                targetPose = new Pose2d(FieldConstants.Tower.oppRightUpright, new Rotation2d());
            } else {
                targetPose = new Pose2d(FieldConstants.Tower.oppLeftUpright, new Rotation2d());
            }
        } else {
            if (drivetrain.getPose().getTranslation().getDistance(FieldConstants.Tower.leftUpright) > drivetrain
                    .getPose().getTranslation().getDistance(FieldConstants.Tower.rightUpright)) {
                targetPose = new Pose2d(FieldConstants.Tower.rightUpright, new Rotation2d());
            } else {
                targetPose = new Pose2d(FieldConstants.Tower.leftUpright, new Rotation2d());
            }
        }

        return targetPose;
    }
}