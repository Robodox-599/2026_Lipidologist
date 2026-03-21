package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.drive.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ChassisAccelerations;
import frc.robot.util.Tracer;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private final double CHOREO_MAX_ERROR_MARGIN = 0.05;
  public static final double DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT = 0.1;
  private final double DRIVE_TO_POINT_MAX_VELOCITY_OUTPUT = 2.0;
  private final double DRIVE_TO_POINT_LINEAR_ERROR_MARGIN = 0.05;
  private final double DRIVE_TO_POINT_ANGULAR_ERROR_MARGIN = Units.degreesToRadians(5);

  CommandXboxController driver;

  private final PIDController driveToPointController = new PIDController(3, 0.0, 0.0); // P: 3.0/3.6, D: 0.1 | TODO:
                                                                                       // change to profiled PID
                                                                                       // controller
  private final PIDController choreoXController = new PIDController(5, 0, 0);
  private final PIDController choreoYController = new PIDController(5, 0, 0);
  private final PIDController choreoThetaPID = new PIDController(5, 0, 0);
  private final PIDController choreoRotationLockPID = new PIDController(5, 0, 0);

  private ChassisSpeeds prevFieldRelVelocities = new ChassisSpeeds();

  private Rotation2d targetRotation = new Rotation2d();
  private Pose2d targetPosition = new Pose2d();

  private SwerveSample choreoSampleToBeApplied;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0)
      .withRotationalDeadband(0) // Add a 10% deadband
      .withDriveRequestType(
          DriveRequestType.Velocity); // Use open-loop control for drive motors

  private final SwerveRequest.FieldCentricFacingAngle driveAtAngle = new SwerveRequest.FieldCentricFacingAngle()
      .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
      .withDriveRequestType(DriveRequestType.Velocity);

  private CurrentState currentState = CurrentState.TELEOP_DRIVE;
  private WantedState wantedState = WantedState.TELEOP_DRIVE;

  public enum WantedState {
    TELEOP_DRIVE,
    ROTATION_LOCK,
    DRIVE_TO_POINT,
    CHOREO_TRAJECTORY,
    ROTATION_LOCK_AND_FOLLOW_CHOREO_TRAJECTORY,
    STOPPED,
  }

  public enum CurrentState {
    TELEOP_DRIVE,
    ROTATION_LOCK,
    DRIVE_TO_POINT,
    CHOREO_TRAJECTORY,
    ROTATION_LOCK_AND_FOLLOW_CHOREO_TRAJECTORY,
    STOPPED,
  }

  private final Telemetry telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
  private double MaxAngularRate = RotationsPerSecond.of(0.75)
      .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

  /*
   * SysId routine for characterizing translation. This is used to find PID gains
   * for the drive motors.
   */
  private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /*
   * SysId routine for characterizing steer. This is used to find PID gains for
   * the steer motors.
   */
  private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(7), // Use dynamic voltage of 7 V
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle
   * HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
   * importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
      new SysIdRoutine.Config(
          /* This is in radians per second², but SysId only supports "volts per second" */
          Volts.of(Math.PI / 6).per(Second),
          /* This is in radians per second, but SysId only supports "volts" */
          Volts.of(Math.PI),
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> {
            /* output is actually radians per second, but SysId only supports "volts" */
            setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
            /* also log the requested output for SysId */
            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
          },
          null,
          this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>
   * This constructs the underlying hardware devices, so users should not
   * construct the devices
   * themselves. If they need the devices, they can access them through getters in
   * the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules             Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      CommandXboxController driver,
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    this.driver = driver;

    driveAtAngle.HeadingController = new PhoenixPIDController(5, 0, 0);
    driveAtAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    choreoThetaPID.enableContinuousInput(-Math.PI, Math.PI);

    this.registerTelemetry(telemetry::telemeterize);
  }

  /**
   * Returns a command that applies the specified control request to this swerve
   * drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine
   * specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified
   * by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  public void updateInputs() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled.
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }
    prevFieldRelVelocities = getFieldRelativeChassisSpeeds();
    Tracer.traceFunc("HandleStateTransitions", this::handleStateTransitions);
    Tracer.traceFunc("ApplyStates", this::applyStates);
    DogLog.log("Drive/CurrentState", currentState);
    DogLog.log("Drive/WantedState", wantedState);
    DogLog.log("RobotPose", getState().Pose);
    // DogLog.log("Drive/IsAtTargetRotation", isAtTargetRotation());
    // DogLog.log("Drive/isAtTargetPose", isAtTargetPose());
    // DogLog.log("Drive/Choreo/IsAtEndOfChoreoTrajectory",
    // isAtEndOfChoreoTrajectory());
  }

  private void handleStateTransitions() {
    switch (wantedState) {
      case TELEOP_DRIVE:
        currentState = CurrentState.TELEOP_DRIVE;
        break;
      case ROTATION_LOCK:
        currentState = CurrentState.ROTATION_LOCK;
        break;
      case DRIVE_TO_POINT:
        currentState = CurrentState.DRIVE_TO_POINT;
        break;
      case CHOREO_TRAJECTORY:
        currentState = CurrentState.CHOREO_TRAJECTORY;
        break;
      case ROTATION_LOCK_AND_FOLLOW_CHOREO_TRAJECTORY:
        currentState = CurrentState.ROTATION_LOCK_AND_FOLLOW_CHOREO_TRAJECTORY;
        break;
      case STOPPED:
        currentState = CurrentState.STOPPED;
        break;
      default:
        currentState = CurrentState.TELEOP_DRIVE;
        break;
    }
  }

  private void applyStates() {
    ChassisSpeeds controllerSpeeds = new ChassisSpeeds(
        //leftY only accounts for the robots translation vertically
        //leftX only accounts for the robots translation horizontally
        //rightX only accounts for the robots rotation
        //all of these represents the X and Y values outputed by the joysticks (left or right)

        //joystickDeadbandApply acounts the wear and tear by the joystick over time. The joystick's value may never truly be at zero
        // so it returns the value of 0 if it's within a certain tolerance

        //driver.getLeftY() returns a value [-1,1]. However, this restricts it's maximum speed to 1. To get our desired maximum speeds,
        //you multiply it by the max Speeds

        //note that this doesn't apply the velocites but only gets the values for it

        -joystickDeadbandApply(driver.getLeftY() * TunerConstants.MaxSpeed),
        -joystickDeadbandApply(driver.getRightX() * TunerConstants.MaxSpeed),
        -joystickDeadbandApply(driver.getRightX() * TunerConstants.MaxSpeed));

        DogLog.log("Drive/RobotSpeed", controllerSpeeds);
    switch (currentState) {
      case STOPPED:
        setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        break;
      case TELEOP_DRIVE:
        //here is where we apply the robots velocity from the controller speeds we got
        setControl(
          drive
            .withVelocityX(controllerSpeeds.vxMetersPerSecond)
            .withVelocityY(controllerSpeeds.vyMetersPerSecond)
            .withRotationalRate(controllerSpeeds.omegaRadiansPerSecond)
        );
        break;
      case ROTATION_LOCK:
        // this case allows the robot to translate freely, but makes it such that the robot's rotation has a target to lock onto.
        // some examples of this case being used is when the robot initially locks to the hub only allowing fuel to be shot then,
        // preventing fuel from being shot from the field. Also because it locks rotation and not movement, it allows you to score
        // on the move
        setControl(
          drive
            .withVelocityX(controllerSpeeds.vxMetersPerSecond)
            .withVelocityY(controllerSpeeds.vyMetersPerSecond)
            .withRotationalRate(targetRotation.getRadians()) //we don't want the rightX to affect the robots rotation but only it's target
        );
        // we don't specify the case whether it's auto or teleop because it can be used for both
        break;
      case DRIVE_TO_POINT:
        // this case allows the robot to move to a specific point, anywhere on the field.
        // for example this case can be used during the end game, where the robot needs to align with the tower for the hook to to latch
        // this must mean that we must use translation and rotation

        // here we do final minus initial to get our displacement from the target position
        // we first get the translation (x, y point) for our target position an duse the .minus() to avoid using object type conversion
        // which makes it more easier to write
        // for our current translation, we get the translation of our current State's Pose
        // but why is Pose capitalized
        Translation2d targetTranslation2d = this.targetPosition.getTranslation().minus(getState().Pose.getTranslation());

        // notice how targetPosition is a Pose2d, so it has attributes of translation and rotation
        Rotation2d rotation2d = this.targetPosition.getRotation();

        // we know that there are 2 sides on the feild, this must mean that depending on which side the robot is on, however, we only "flip"
        // when the robot is on the red alliance
        if (AllianceFlipUtil.shouldFlip()){ //shouldFlip returns true if the robot is on the red alliance
          targetTranslation2d = targetTranslation2d.rotateBy(kRedAlliancePerspectiveRotation);
        }

        // this tells the robot how faw away it is in a straight line using pythagorean theorm.
        // but why not use target Translation 2d? It's because the type is Translation2d and not a double. We need the type to be
        // a double because we'll need it for the PID controller
        double linearDistance = targetTranslation2d.getNorm(); // the norm does pythagoreon theorm and returns the double


        // this isto compensate for friction when the robot is close to our target. This allwos the swerve to not stall once it
        // gets near
        double frictionBoost = 0.0;
        if (linearDistance >= 0.02){
          frictionBoost = DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT * TunerConstants.MaxSpeed;
        }

        //Calculates velocity output based on PID and distance including static friction.
        // It also ensures that it doesn't go over the limit
        double velocityOutput = Math.min(
          Math.abs(driveToPointController.calculate(linearDistance, 0)) + frictionBoost, DRIVE_TO_POINT_MAX_VELOCITY_OUTPUT);

        // now let's get the horizontal and veticle components of the linearVelocity so we can apply it to our control as well
        // as the angle we want to set the robot to
        Rotation2d driveDirection = targetTranslation2d.getAngle();
        double xVelocity = velocityOutput * driveDirection.getCos();
        double yVelocity = velocityOutput * driveDirection.getSin();

        //Apply the velocities and angle now
        setControl(
          driveAtAngle
          .withVelocityX(xVelocity)
          .withVelocityY(yVelocity)
          .withTargetDirection(rotation2d)
        );
        break;
      case CHOREO_TRAJECTORY:
        if (choreoSampleToBeApplied){}
        }

      default:
        break;
    }
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public void setWantedState(WantedState wantedState, Rotation2d targetRotation) {
    this.wantedState = wantedState;
    this.targetRotation = targetRotation;
    if (AllianceFlipUtil.shouldFlip()) {
      this.targetRotation = targetRotation.rotateBy(kRedAlliancePerspectiveRotation);
    }
  }

  public void setWantedState(WantedState wantedState, Pose2d targetPosition) {
    this.wantedState = wantedState;
    this.targetPosition = targetPosition;
    // if (AllianceFlipUtil.shouldFlip()) {
    // this.targetPosition =
    // targetPosition.rotateBy(kRedAlliancePerspectiveRotation);
    // }
  }

  public void setTargetRotation(Rotation2d targetRotation) {
    this.targetRotation = targetRotation;
    if (AllianceFlipUtil.shouldFlip()) {
      this.targetRotation = targetRotation.rotateBy(kRedAlliancePerspectiveRotation);
    }
  }

  // public void setWantedState(WantedState wantedState, Pose2d targetPose) {
  // this.wantedState = wantedState;
  // this.targetRotation = targetRotation;
  // if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
  // this.targetRotation = targetRotation.rotateBy(Rotation2d.k180deg);
  // }
  // }

  public boolean isAtTargetRotation() {
    return driveAtAngle.HeadingController.getPositionError() < Units.degreesToRadians(3);
  }

  public boolean isAtTargetPose() {
    return MathUtil.isNear(0.0, this.targetPosition.getTranslation().minus(getState().Pose.getTranslation()).getNorm(),
        DRIVE_TO_POINT_LINEAR_ERROR_MARGIN)
        && MathUtil.isNear(
            getState().Pose.getRotation().getRadians(),
            this.targetPosition.getRotation().getRadians(),
            DRIVE_TO_POINT_ANGULAR_ERROR_MARGIN);
  }

  public boolean isAtTargetPose(Pose2d targetPose) {
    return MathUtil.isNear(0.0, targetPose.getTranslation().minus(getState().Pose.getTranslation()).getNorm(),
        DRIVE_TO_POINT_LINEAR_ERROR_MARGIN)
        && MathUtil.isNear(
            getState().Pose.getRotation().getRadians(),
            targetPose.getRotation().getRadians(),
            DRIVE_TO_POINT_ANGULAR_ERROR_MARGIN);
  }

  /* CHOREO TRAJECTORY */
  public void setDesiredChoreoTrajectory(SwerveSample sample) {
    this.choreoSampleToBeApplied = sample;
  }

  /* DATA */
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return getState().Speeds;
  }

  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), getPose().getRotation()); // rotation
                                                                                                            // may need
                                                                                                            // to be
                                                                                                            // flipped?
  }

  public ChassisAccelerations getFieldRelativeAccelerations() {
    return new ChassisAccelerations(getFieldRelativeChassisSpeeds(), prevFieldRelVelocities, 0.020);
  }

  public Pose2d getPose() {
    return getState().Pose;
  }

  /* GYRO */
  public void zeroGyro() {
    resetRotation(new Rotation2d(0.0));
  }

  public Command zeroGyroCommand() {
    return new InstantCommand(
        () -> {
          resetRotation(new Rotation2d(0.0));
        });
  }

  /* JOYSTICK DEADBAND */
  public double joystickDeadbandApply(double x) {
    return MathUtil.applyDeadband(
        (Math.signum(x) * (1.01 * Math.pow(x, 2) - 0.0202 * x + 0.0101)), 0.02);
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(
        () -> {
          final double currentTime = Utils.getCurrentTimeSeconds();
          double deltaTime = currentTime - m_lastSimTime;
          m_lastSimTime = currentTime;

          /* use the measured time delta, get battery voltage from WPILib */
          updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the
   * odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision
   *                              camera.
   * @param timestampSeconds      The timestamp of the vision measurement in
   *                              seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the
   * odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>
   * Note that the vision measurement standard deviations passed into this method
   * will continue
   * to apply to future measurements until a subsequent call to {@link
   * #setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters    The pose of the robot as measured by the
   *                                 vision camera.
   * @param timestampSeconds         The timestamp of the vision measurement in
   *                                 seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose
   *                                 measurement in the form
   *                                 [x, y, theta]ᵀ, with units in meters and
   *                                 radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }
}
