package frc.robot;

import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.vision6.Vision;
import frc.robot.subsystems.vision6.VisionConstants;
import frc.robot.subsystems.vision6.VisionIOReal;
import frc.robot.subsystems.vision6.VisionIOSim;
import frc.robot.util.HubShiftUtil;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.flywheels.FlywheelsConstants;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIOSim;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIOTalonFX;
import frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.FlywheelConstants;

import java.lang.reflect.Field;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.autos.AutoRoutines;
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
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollersIOSim;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.intake.intakeWrist.IntakeWrist;
import frc.robot.subsystems.intake.intakeWrist.IntakeWristIOSim;
import frc.robot.subsystems.intake.intakeWrist.IntakeWristIOTalonFX;

public class Robot extends TimedRobot {
  private final CommandScheduler scheduler = CommandScheduler.getInstance();

  final CommandXboxController driver = new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
  // final CommandXboxController operator = new CommandXboxController(
  // Constants.ControllerConstants.kOperatorControllerPort);
  // final Climb climb;
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

  final AutoChooser autoChooser = new AutoChooser();
  final AutoFactory autoFactory;
  final AutoRoutines autoRoutines;

  @Override
  protected void loopFunc() {
    super.loopFunc();
  }

  public Robot() {
    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withCaptureNt(true)
            .withNtPublish(true)
            .withCaptureConsole(true));

    switch (Constants.currentMode) {
      case REAL:
        // climb = new Climb(new ClimbIOTalonFX());
        drivetrain = TunerConstants.createDrivetrain(driver);
        feeder = new Feeder(new FeederIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        intakeRollers = new IntakeRollers(new IntakeRollersIOTalonFX());
        intakeWrist = new IntakeWrist(new IntakeWristIOTalonFX());
        flywheels = new Flywheels(new FlywheelsIOTalonFX(FlywheelsConstants.LeftFlywheel),
            new FlywheelsIOTalonFX(FlywheelsConstants.MiddleFlywheel),
            new FlywheelsIOTalonFX(FlywheelsConstants.RightFlywheel));
        hood = new Hood(new HoodIOTalonFX());
        vision = new Vision(drivetrain::addVisionMeasurement,
            new VisionIOReal(VisionConstants.frontLeftCameraConstants, () -> drivetrain.getPose()),
            new VisionIOReal(VisionConstants.frontRightCameraConstants, () -> drivetrain.getPose()));
        leds = new LEDs(new LEDsIOReal());
        break;
      case SIM:
        // climb = new Climb(new ClimbIOSim());
        drivetrain = TunerConstants.createDrivetrain(driver);
        feeder = new Feeder(new FeederIOSim());
        indexer = new Indexer(new IndexerIOSim());
        intakeRollers = new IntakeRollers(new IntakeRollersIOSim());
        intakeWrist = new IntakeWrist(new IntakeWristIOSim());
        flywheels = new Flywheels(new FlywheelsIOSim(FlywheelsConstants.LeftFlywheelSim));
        hood = new Hood(new HoodIOSim());
        vision = new Vision(drivetrain::addVisionMeasurement,
            new VisionIOSim(VisionConstants.frontLeftCameraConstants, () -> drivetrain.getPose()),
            new VisionIOSim(VisionConstants.frontRightCameraConstants, () -> drivetrain.getPose()));
            leds = new LEDs(new LEDsIO());
        break;
      default: // defaults to sim
        // climb = new Climb(new ClimbIOSim());
        drivetrain = TunerConstants.createDrivetrain(driver);
        feeder = new Feeder(new FeederIOSim());
        indexer = new Indexer(new IndexerIOSim());
        intakeRollers = new IntakeRollers(new IntakeRollersIOSim());
        intakeWrist = new IntakeWrist(new IntakeWristIOSim());
        flywheels = new Flywheels(new FlywheelsIOSim(FlywheelsConstants.LeftFlywheelSim));
        hood = new Hood(new HoodIOSim());
        vision = new Vision(drivetrain::addVisionMeasurement,
            new VisionIOSim(VisionConstants.frontLeftCameraConstants, () -> drivetrain.getPose()),
            new VisionIOSim(VisionConstants.frontRightCameraConstants, () -> drivetrain.getPose()));
            leds = new LEDs(new LEDsIO());
        break;
    }

    superstructure = new Superstructure(
        // climb,
        drivetrain,
        feeder, indexer,
        intakeRollers, intakeWrist,
        flywheels, hood,
        vision, leds);

    new Bindings(driver, superstructure);

    autoFactory = new AutoFactory(
        drivetrain::getPose,
        drivetrain::resetPose,
        drivetrain::setDesiredChoreoTrajectory,
        true,
        drivetrain);

    autoRoutines = new AutoRoutines(autoFactory, superstructure, drivetrain);

    // Auto chooser setup
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    /** AUTO ROUTINES */
    // COMPETITION
    autoChooser.addRoutine("Left Auto", autoRoutines::leftAutoRoutine);

    SmartDashboard.putData("AutoChooser", autoChooser);

    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> HubShiftUtil.initialize()));
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> HubShiftUtil.initialize()));

    SmartDashboard.putNumber("Flywheel Velocity", 0.0);
  }

  @Override
  public void robotPeriodic() {
    flywheels.setRPS(SmartDashboard.getNumber("Flywheel Velocity", 0.0));
    scheduler.run();
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
