package frc.robot;

import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood.HoodWantedState;
import frc.robot.subsystems.vision6.Vision;
import frc.robot.subsystems.vision6.VisionConstants;
import frc.robot.subsystems.vision6.VisionIOReal;
import frc.robot.subsystems.vision6.VisionIOSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.Tracer;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.flywheels.FlywheelsConstants;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIOSim;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIOTalonFX;
import frc.robot.subsystems.shooter.flywheels.Flywheels.FlywheelWantedState;
import frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.FlywheelConstants;
import frc.robot.autos.AutoBuilder;

import java.lang.reflect.Field;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers.IntakeRollersWantedState;
import frc.robot.subsystems.intake.intakeWrist.IntakeWrist;
import frc.robot.subsystems.intake.intakeWrist.IntakeWristIOTalonFX;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDsIO;
import frc.robot.subsystems.leds.LEDsIOReal;
import frc.robot.subsystems.intake.intakeWrist.IntakeWrist.IntakeWristWantedState;
import frc.robot.FieldConstants.LeftTrench;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain.WantedState;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.feeder.Feeder.FeederWantedState;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.indexer.Indexer.IndexerWantedState;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollersIOSim;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.intake.intakeWrist.IntakeWrist;
import frc.robot.subsystems.intake.intakeWrist.IntakeWristIOSim;
import frc.robot.subsystems.intake.intakeWrist.IntakeWristIOTalonFX;

public class Robot extends TimedRobot {
  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  private Command autonomousCommand;

  final CommandXboxController driver = new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
  // final CommandXboxController operator = new CommandXboxController(
  // Constants.ControllerConstants.kOperatorControllerPort);
  final Climb climb;
  final CommandSwerveDrivetrain drivetrain;
  final Feeder feeder;
  final Indexer indexer;
  final IntakeRollers intakeRollers;
  final IntakeWrist intakeWrist;
  final Flywheels flywheels;
  final Hood hood;
  final Vision vision;
  final LEDs leds;
  final Superstructure superstructure;

  // final AutoChooser autoChooser = new AutoChooser();
  // final AutoFactory autoFactory;
  // final AutoCommands autoRoutines;

  @Override
  protected void loopFunc() {
    Tracer.startTrace("RobotLoop");
    super.loopFunc();
    Tracer.endTrace();
  }

  public Robot() {
    Tracer.enableSingleThreadedMode();
    Tracer.enableTracingForCurrentThread();

    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withCaptureNt(true)
            .withNtPublish(true)
            .withCaptureConsole(true));

    switch (Constants.currentMode) {
      case REAL:
        climb = new Climb(new ClimbIOTalonFX());
        drivetrain = TunerConstants.createDrivetrain(driver);
        feeder = new Feeder(new FeederIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        intakeRollers = new IntakeRollers(new IntakeRollersIOTalonFX());
        intakeWrist = new IntakeWrist(new IntakeWristIOTalonFX());
        flywheels = new Flywheels(new FlywheelsIOTalonFX(FlywheelsConstants.LeftFlywheel),
            new FlywheelsIOTalonFX(FlywheelsConstants.MiddleFlywheel),
            new FlywheelsIOTalonFX(FlywheelsConstants.RightFlywheel));
        hood = new Hood(new HoodIOTalonFX());
        vision = new Vision(drivetrain::addVisionMeasurement, () -> drivetrain.getFieldRelativeChassisSpeeds(),
            new VisionIOReal(VisionConstants.frontLeftCameraConstants, () -> drivetrain.getPose()),
            new VisionIOReal(VisionConstants.frontRightCameraConstants, () -> drivetrain.getPose()));
        leds = new LEDs(new LEDsIOReal());
        break;
      case SIM:
        climb = new Climb(new ClimbIOSim());
        drivetrain = TunerConstants.createDrivetrain(driver);
        feeder = new Feeder(new FeederIOSim());
        indexer = new Indexer(new IndexerIOSim());
        intakeRollers = new IntakeRollers(new IntakeRollersIOSim());
        intakeWrist = new IntakeWrist(new IntakeWristIOSim());
        flywheels = new Flywheels(new FlywheelsIOSim(FlywheelsConstants.LeftFlywheelSim));
        hood = new Hood(new HoodIOSim());
        vision = new Vision(drivetrain::addVisionMeasurement, () -> drivetrain.getFieldRelativeChassisSpeeds(),
            new VisionIOSim(VisionConstants.frontLeftCameraConstants, () -> drivetrain.getPose()),
            new VisionIOSim(VisionConstants.frontRightCameraConstants, () -> drivetrain.getPose()));
        leds = new LEDs(new LEDsIO());
        break;
      default: // defaults to sim
        climb = new Climb(new ClimbIOSim());
        drivetrain = TunerConstants.createDrivetrain(driver);
        feeder = new Feeder(new FeederIOSim());
        indexer = new Indexer(new IndexerIOSim());
        intakeRollers = new IntakeRollers(new IntakeRollersIOSim());
        intakeWrist = new IntakeWrist(new IntakeWristIOSim());
        flywheels = new Flywheels(new FlywheelsIOSim(FlywheelsConstants.LeftFlywheelSim));
        hood = new Hood(new HoodIOSim());
        vision = new Vision(drivetrain::addVisionMeasurement, () -> drivetrain.getFieldRelativeChassisSpeeds(),
            new VisionIOSim(VisionConstants.frontLeftCameraConstants, () -> drivetrain.getPose()),
            new VisionIOSim(VisionConstants.frontRightCameraConstants, () -> drivetrain.getPose()));
        leds = new LEDs(new LEDsIO());
        break;
    }

    superstructure = new Superstructure(
        climb,
        drivetrain,
        feeder, indexer,
        intakeRollers, intakeWrist,
        flywheels, hood,
        vision, leds);

    new Bindings(driver, superstructure);

    // autoFactory = new AutoFactory(
    //     drivetrain::getPose,
    //     drivetrain::resetPose,
    //     drivetrain::setDesiredChoreoTrajectory,
    //     true,
    //     drivetrain);

    // autoRoutines = new AutoRoutines(autoFactory, superstructure, drivetrain);
    // Auto chooser setup
    // RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    
    // /** AUTO ROUTINES */
    // // COMPETITION
    // autoChooser.addRoutine("Left Auto", autoRoutines::leftAutoRoutine);
    // autoChooser.addRoutine("Right Sweep Auto", autoRoutines::sweepAutoRoutine);
    // SmartDashboard.putData("AutoChooser", autoChooser);

    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> HubShiftUtil.initialize()));
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> HubShiftUtil.initialize()));

    // SmartDashboard.putNumber("Flywheel Velocity", 0.0);
    // SmartDashboard.putNumber("Hood Rotations", 0.0);

    SmartDashboard.putNumber("VisionLinearStdDev", 0.05);
    SmartDashboard.putNumber("VisionAngularStdDev", 0.01);

    DogLog.log("LeftTrenchZone", FieldConstants.LeftTrench.trenchZone);
    DogLog.log("RightTrenchZone", FieldConstants.RightTrench.trenchZone);

  }

  private void configureAutos(CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
    AutoBuilder autoBuilder = new AutoBuilder(drivetrain, superstructure);



  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }

  public CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
  }

  @Override
  public void robotPeriodic() {
    Tracer.traceFunc("CommandScheduler", scheduler::run);
    VisionConstants.linearStdDevBaseline = SmartDashboard.getNumber("VisionLinearStdDev", 0.0);
    VisionConstants.angularStdDevBaseline = SmartDashboard.getNumber("VisionAngularStdDev", 0.0);
    // flywheels.setWantedState(FlywheelWantedState.SET_RPS,
    // SmartDashboard.getNumber("Flywheel Velocity", 0.0));
    // hood.setWantedState(HoodWantedState.SET_POSITION,
    // SmartDashboard.getNumber("Hood Rotations", 0.0));
    // feeder.setWantedState(FeederWantedState.FEED_FUEL);
    // indexer.setWantedState(IndexerWantedState.TRANSFER_FUEL);
    // intakeWrist.setWantedState(IntakeWristWantedState.INTAKE_FUEL);

    Translation2d robotTranslation = drivetrain.getPose().getTranslation();
    Translation2d hubTranslation = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    double distance = robotTranslation.getDistance(hubTranslation);
    DogLog.log("DistanceToHub", distance);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
    scheduler.cancelAll();
  }

  @Override
  public void autonomousInit() {
    // autonomousCommand = AutoBuilder.leftFarMidDepotAuto();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    scheduler.cancelAll();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
    scheduler.cancelAll();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
    scheduler.cancelAll();
  }
}
