package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
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
import frc.robot.util.CalculateShot;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.HubShiftUtil.ShiftInfo;
import frc.robot.util.ShotData;
import frc.robot.util.CalculateShot.AdjustedShot;
import frc.robot.util.FuelSim;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.util.AllianceFlipUtil;

public class Superstructure extends SubsystemBase {

    // final Climb climb;
    final CommandSwerveDrivetrain drivetrain;
    // final Feeder feeder;
    // final Indexer indexer;
    final IntakeRollers intakeRollers;
    final IntakeWrist intakeWrist;
    // final Flywheels flywheels;
    // final Hood hood;
    final Vision vision;
    final FuelSim fuelSim;
    private final ShotData shotCalculator = new ShotData();

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
            // Climb climb,
            CommandSwerveDrivetrain drivetrain,
            // Feeder feeder,
            // Indexer indexer,
            IntakeRollers intakeRollers,
            IntakeWrist intakeWrist,
            // Flywheels flywheels,
            // Hood hood
            Vision vision,
            FuelSim fuelSim) {
        // this.climb = climb;
        this.drivetrain = drivetrain;
        // this.feeder = feeder;
        // this.indexer = indexer;
        this.intakeRollers = intakeRollers;
        this.intakeWrist = intakeWrist;
        // this.flywheels = flywheels;
        // this.hood = hood;
        this.vision = vision;
        this.fuelSim = fuelSim;
    }

    @Override
    public void periodic() {
        // climb.updateInputs();
        drivetrain.updateInputs();
        // feeder.updateInputs();
        // indexer.updateInputs();
        intakeRollers.updateInputs();
        intakeWrist.updateInputs();
        // flywheels.updateInputs();
        // hood.updateInputs();
        vision.updateInputs();

        fuelSim.updateSim();

        ShiftInfo shiftInfo = HubShiftUtil.getShiftInfo();
        DogLog.log("HubShift", shiftInfo.currentShift());
        DogLog.log("HubShift/ElapsedTime", shiftInfo.elapsedTime());
        DogLog.log("HubShift/RemainingTime", shiftInfo.remainingTime());
        DogLog.log("HubShift/Active", shiftInfo.active());

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
                // if (areSystemsReadyForHubShot()) {
                currentSuperState = CurrentSuperState.SHOOTING_HUB;
                // } else {
                // currentSuperState = CurrentSuperState.PREPARING_HUB_SHOT;
                // }
                break;
            case PREPARE_ALLIANCE_ZONE_SHOT:
                currentSuperState = CurrentSuperState.PREPARING_ALLIANCE_ZONE_SHOT;
                break;
            case SHOOT_ALLIANCE_ZONE:
                // if (areSystemsReadyForAllianceZoneShot()) {
                currentSuperState = CurrentSuperState.SHOOTING_ALLIANCE_ZONE;
                // } else {
                // currentSuperState = CurrentSuperState.PREPARING_ALLIANCE_ZONE_SHOT;
                // }
                break;
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
            case IDLING:
                idling();
                break;
            case STOPPED:
                stop();
                break;
        }
    }

    public void preparingHubShot() {
        AdjustedShot adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
                drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());

        drivetrain.setTargetRotation(adjustedShot.targetRotation());

        // Translation2d hubTranslation =
        // FieldConstants.Hub.topCenterPoint.toTranslation2d();
        // Translation2d robotTranslation = drivetrain.getPose().getTranslation();
        // Rotation2d targetRotation = Rotation2d
        // .fromRadians(Math.atan2(hubTranslation.getY() - robotTranslation.getY(),
        // hubTranslation.getX() - robotTranslation.getX()));
        // drivetrain.setTargetRotation(targetRotation);

        // feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        // indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPM,
        // adjustedShot.shootSpeed());
        // hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
        // adjustedShot.hoodAngle());
    }

    public void shootingHub() {
        AdjustedShot adjustedShot = CalculateShot.calculateHubAdjustedShot(drivetrain.getPose(),
                drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());

        drivetrain.setTargetRotation(adjustedShot.targetRotation());

        fuelSim.launchFuel(MetersPerSecond.of((adjustedShot.shootSpeed() / 2.0) * (2.0 * Math.PI * (0.0381))),
                Radians.of(adjustedShot.hoodAngle() * (Math.PI / 180)), Radians.of(Math.PI / 2), Meters.of(0.55));

        // drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
        // feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        // indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPM,
        // adjustedShot.shootSpeed());
        // hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
        // adjustedShot.hoodAngle());
    }

    public void preparingAllianceZoneShot() {
        AdjustedShot adjustedShot = CalculateShot.calculateAllianceZoneAdjustedShot(drivetrain.getPose(),
                drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());

        drivetrain.setTargetRotation(adjustedShot.targetRotation());
        // feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        // indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPM,
        // adjustedShot.shootSpeed());
        // hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
        // adjustedShot.hoodAngle());
    }

    public void shootingAllianceZone() {
        AdjustedShot adjustedShot = CalculateShot.calculateAllianceZoneAdjustedShot(drivetrain.getPose(),
                drivetrain.getFieldRelativeChassisSpeeds(), drivetrain.getFieldRelativeAccelerations());

        drivetrain.setTargetRotation(adjustedShot.targetRotation());
        // feeder.setWantedState(Feeder.FeederWantedState.FEED_FUEL);
        // indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.AGITATE_FUEL);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPM,
        // adjustedShot.shootSpeed());
        // hood.setWantedState(Hood.HoodWantedState.SET_POSITION,
        // adjustedShot.hoodAngle());
    }

    public void idling() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.TELEOP_DRIVE);
        // feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        // indexer.setWantedState(Indexer.IndexerWantedState.TRANSFER_FUEL);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.INTAKE_FUEL);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.INTAKE_FUEL);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.SET_RPM);
        // hood.setWantedState(Hood.HoodWantedState.SET_POSITION);
    }

    public void stop() {
        drivetrain.setWantedState(CommandSwerveDrivetrain.WantedState.STOPPED);
        // feeder.setWantedState(Feeder.FeederWantedState.STOPPED);
        // indexer.setWantedState(Indexer.IndexerWantedState.STOPPED);
        intakeRollers.setWantedState(IntakeRollers.IntakeRollersWantedState.STOP);
        intakeWrist.setWantedState(IntakeWrist.IntakeWristWantedState.STOP);
        // flywheels.setWantedState(Flywheels.FlywheelWantedState.STOPPED);
        // hood.setWantedState(Hood.HoodWantedState.STOPPED);
    }

    // private boolean areSystemsReadyForHubShot() {
    // return flywheels.atSetpoint() && hood.atSetpoint() &&
    // drivetrain.isAtTargetRotation() && HubShiftUtil.isHubActive();
    // }

    // private boolean areSystemsReadyForAllianceZoneShot() {
    // return flywheels.atSetpoint() && hood.atSetpoint() &&
    // drivetrain.isAtTargetRotation();
    // }

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
