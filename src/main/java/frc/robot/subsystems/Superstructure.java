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
import frc.robot.subsystems.feeder.Feeder.FeederWantedState;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.intake.intakeWrist.IntakeWrist;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.vision6.Vision;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.util.CalculateShot;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.HubShiftUtil.ShiftInfo;
import frc.robot.util.ShotData;
import frc.robot.util.Tracer;
import frc.robot.util.CalculateShot.AdjustedShot;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.flywheels.Flywheels.FlywheelWantedState;
import frc.robot.util.AllianceFlipUtil;

public class Superstructure extends SubsystemBase {

    // final Climb climb;
    final CommandSwerveDrivetrain drivetrain;
    final Feeder feeder;
    final Indexer indexer;
    // final IntakeRollers intakeRollers;
    // final IntakeWrist intakeWrist;
    final Flywheels flywheels;
    final Hood hood;
    final LEDs leds;
    final Vision vision;
    private final ShotData shotCalculator = new ShotData();

    public enum WantedSuperState {
        PREPARE_HUB_SHOT,
        PREPARE_HUB_SHOT_ALIGNED_TROUGH,
        PREPARE_ALLIANCE_ZONE_SHOT,
        PREPARE_ALLIANCE_ZONE_SHOT_ALIGNED_TROUGH,
        SHOOT_HUB,
        SHOOT_HUB_AUTO,
        SHOOT_ALLIANCE_ZONE,
        IDLE,
        IDLE_AUTO,
        CLIMB,
        STOP,
        TESTING
    }

    public enum CurrentSuperState {
        PREPARING_HUB_SHOT,
        PREPARING_HUB_SHOT_AUTO,
        PREPARING_HUB_SHOT_ALIGNED_TROUGH,
        PREPARING_ALLIANCE_ZONE_SHOT,
        SHOOTING_HUB,
        SHOOTING_HUB_AUTO,
        SHOOTING_ALLIANCE_ZONE,
        IDLING,
        IDLING_AUTO,
        AUTO_ALIGN_CLIMB,
        EXTEND_CLIMB,
        RETRACT_CLIMB,
        STOPPED,
        TESTING
    }

    private WantedSuperState wantedSuperState = WantedSuperState.STOP;
    private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;

    public Superstructure(
            // Climb climb,
            CommandSwerveDrivetrain drivetrain,
            Feeder feeder,
            Indexer indexer,
            // IntakeRollers intakeRollers,
            // IntakeWrist intakeWrist,
            Flywheels flywheels,
            Hood hood,
            Vision vision,
            LEDs leds) {
        // this.climb = climb;
        this.drivetrain = drivetrain;
        this.feeder = feeder;
        this.indexer = indexer;
        // this.intakeRollers = intakeRollers;
        // this.intakeWrist = intakeWrist;
        this.flywheels = flywheels;
        this.hood = hood;
        this.vision = vision;
        this.leds = leds;
    }

    @Override
    public void periodic() {
        Tracer.startTrace("SuperstructurePeriodic");

        // climb.updateInputs();
        Tracer.traceFunc("Drivetrain Periodic", drivetrain::updateInputs);
        Tracer.traceFunc("FeederPeriodic", feeder::updateInputs);
        Tracer.traceFunc("IndexerPeriodic", indexer::updateInputs);
        // Tracer.traceFunc("IntakeRollers Periodic", intakeRollers::updateInputs);
        // Tracer.traceFunc("IntakeWrist Periodic", intakeWrist::updateInputs);
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

        Tracer.endTrace();
    }

    private void handleStateTransitions() {
        switch (wantedSuperState) {
            default:
            case PREPARE_HUB_SHOT:
                currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT;
                break;
            case SHOOT_HUB:
                if (areSystemsReadyForHubShot()) {
                    currentSuperState = CurrentSuperState.SHOOTING_HUB;
                } else {
                    currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT;
                }
                break;
            case SHOOT_HUB_AUTO:
                if (areSystemsReadyForHubShot()) {
                    currentSuperState = CurrentSuperState.SHOOTING_HUB_AUTO;
                } else {
                    currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT_AUTO;
                }
                break;
            case PREPARE_ALLIANCE_ZONE_SHOT:
                currentSuperState = CurrentSuperState.PREPARING_ALLIANCE_ZONE_SHOT;
                break;
            case SHOOT_ALLIANCE_ZONE:
                if (areSystemsReadyForAllianceZoneShot()) {
                    currentSuperState = CurrentSuperState.SHOOTING_ALLIANCE_ZONE;
                } else {
                    currentSuperState = CurrentSuperState.PREPARING_ALLIANCE_ZONE_SHOT;
                }
                break;
            case IDLE:
                currentSuperState = CurrentSuperState.IDLING;
                break;
            case IDLE_AUTO:
                currentSuperState = CurrentSuperState.IDLING_AUTO;
                break;
            case STOP:
                currentSuperState = CurrentSuperState.STOPPED;
                break;
            case TESTING:
                currentSuperState = CurrentSuperState.TESTING;
                break;
        }
    }

    private void applyStates() {
        switch (currentSuperState) {
            default:
            case PREPARING_HUB_SHOT:
                preparingHubShot();
                break;
            case PREPARING_HUB_SHOT_AUTO:
                preparingHubShotAuto();
                break;
            case SHOOTING_HUB:
                shootingHub();
                break;
            case SHOOTING_HUB_AUTO:
                shootingHubAuto();
                break;
            case PREPARING_ALLIANCE_ZONE_SHOT:
                preparingAllianceZoneShot();
                break;
            case SHOOTING_ALLIANCE_ZONE:
                shootingAllianceZone();
                break;
            case IDLING:
                idling();
                break;
            case IDLING_AUTO:
                idlingAuto();
                break;
            case STOPPED:
                stop();
                break;
            case TESTING:
                testing();
                break;
        }
    }

    public void preparingHubShot() {
        AdjustedShot adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
                drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());

        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK, adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        // indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        // intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        // intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS, adjustedShot.shootSpeed());
        hood.setWantedState(Hood.HoodWantedState.SET_POSITION, adjustedShot.hoodAngle());
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_HUB_SHOT);
    }

    public void preparingHubShotAuto() {
        AdjustedShot adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
                drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());

        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK_AND_FOLLOW_CHOREO_TRAJECTORY, adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        // indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        // intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        // intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS, adjustedShot.shootSpeed());
        hood.setWantedState(Hood.HoodWantedState.SET_POSITION, adjustedShot.hoodAngle());
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_HUB_SHOT);
    }

    public void shootingHub() {
        AdjustedShot adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
                drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());

        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK, adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        // indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        // intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        // intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS, adjustedShot.shootSpeed());
        hood.setWantedState(Hood.HoodWantedState.SET_POSITION, adjustedShot.hoodAngle());
        leds.setWantedState(LEDs.LEDsWantedState.SHOOT_HUB);
    }

    public void shootingHubAuto() {
        AdjustedShot adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
                drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());

        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK_AND_FOLLOW_CHOREO_TRAJECTORY, adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        // indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        // intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        // intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS, adjustedShot.shootSpeed());
        hood.setWantedState(Hood.HoodWantedState.SET_POSITION, adjustedShot.hoodAngle());
        leds.setWantedState(LEDs.LEDsWantedState.SHOOT_HUB);
    }

    public void preparingAllianceZoneShot() {
        AdjustedShot adjustedShot = CalculateShot.calculateAllianceZoneAdjustedShot(drivetrain.getPose(),
                drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());

        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK, adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        // indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        // intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        // intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS, adjustedShot.shootSpeed());
        hood.setWantedState(Hood.HoodWantedState.SET_POSITION, adjustedShot.hoodAngle());
        leds.setWantedState(LEDs.LEDsWantedState.PREPARE_ALLIANCE_ZONE_SHOT);
    }

    public void shootingAllianceZone() {
        AdjustedShot adjustedShot = CalculateShot.calculateAllianceZoneAdjustedShot(drivetrain.getPose(),
                drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());

        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.ROTATION_LOCK, adjustedShot.targetRotation());
        feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        // indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        // intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        // intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS, adjustedShot.shootSpeed());
        hood.setWantedState(Hood.HoodWantedState.SET_POSITION, adjustedShot.hoodAngle());
        leds.setWantedState(LEDs.LEDsWantedState.SHOOT_ALLIANCE_ZONE);
    }

    public void idling() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
        feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        // indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        // intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        // intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.IDLE);
        hood.setWantedState(Hood.HoodWantedState.SET_POSITION);
        leds.setWantedState(LEDs.LEDsWantedState.IDLE);
    }
    
    public void idlingAuto() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.CHOREO_TRAJECTORY);
        feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        // indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        // intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        // intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS);
        hood.setWantedState(Hood.HoodWantedState.SET_POSITION);
        leds.setWantedState(LEDs.LEDsWantedState.IDLE);
    }

    public void stop() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
        feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        indexer.setWantedState(Indexer.IndexerWantedState.STOPPED);
        // intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.STOP);
        // intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.STOP);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.STOPPED);
        hood.setWantedState(Hood.HoodWantedState.STOPPED);
        leds.setWantedState(LEDs.LEDsWantedState.STOP);
    }

    public void testing() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
        feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        // indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        // intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        // intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPS, 95);
        hood.setWantedState(Hood.HoodWantedState.SET_POSITION, 0.1);
    }

    private boolean areSystemsReadyForHubShot() {
        return flywheels.atSetpoint() && hood.atSetpoint() && drivetrain.isAtTargetRotation() && HubShiftUtil.isHubActive();
    }

    private boolean areSystemsReadyForAllianceZoneShot() {
        return flywheels.atSetpoint() && hood.atSetpoint() && drivetrain.isAtTargetRotation();
    }

    public Command zeroGyroCommand() {
        return this.runOnce(() -> drivetrain.zeroGyro());
    }

    public Command zeroPoseCommand() {
        return this.runOnce(() -> {
            drivetrain.zeroGyro();
            drivetrain.resetPose();
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
