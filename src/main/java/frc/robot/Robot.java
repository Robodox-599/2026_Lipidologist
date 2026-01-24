package frc.robot;

import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.camera.Camera;
import frc.robot.subsystems.vision.camera.CameraIOReal;
import frc.robot.subsystems.vision.camera.CameraTransforms;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIOTalonFX;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.intake.intakeWrist.IntakeWrist;
import frc.robot.subsystems.intake.intakeWrist.IntakeWristIOTalonFX;
import frc.robot.autos.AutoRoutines;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollersIOTalonFX;

public class Robot extends TimedRobot {
  private final CommandScheduler scheduler = CommandScheduler.getInstance();

  final CommandXboxController driver = new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
  final CommandXboxController operator = new CommandXboxController(
      Constants.ControllerConstants.kOperatorControllerPort);
  final Climb climb;
  final CommandSwerveDrivetrain drivetrain;
  final Feeder feeder;
  final Indexer indexer;
  final IntakeRollers intakeRollers;
  final IntakeWrist intakeWrist;
  final Flywheels flywheels;
  final Hood hood;
  final Vision vision;
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
        climb = new Climb(new ClimbIOTalonFX());
        drivetrain = TunerConstants.createDrivetrain(driver);
        feeder = new Feeder(new FeederIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        intakeRollers = new IntakeRollers(new IntakeRollersIOTalonFX());
        intakeWrist = new IntakeWrist(new IntakeWristIOTalonFX());
        flywheels = new Flywheels(new FlywheelsIOTalonFX());
        hood = new Hood(new HoodIOTalonFX());
        vision = new Vision(
            new Camera(new CameraIOReal(CameraTransforms.frontLeftCameraConstants), drivetrain::addVisionMeasurement));
        break;
      default:
        climb = new Climb(new ClimbIOTalonFX());
        drivetrain = TunerConstants.createDrivetrain(driver);
        feeder = new Feeder(new FeederIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        intakeRollers = new IntakeRollers(new IntakeRollersIOTalonFX());
        intakeWrist = new IntakeWrist(new IntakeWristIOTalonFX());
        flywheels = new Flywheels(new FlywheelsIOTalonFX());
        hood = new Hood(new HoodIOTalonFX());
        vision = new Vision(
            new Camera(new CameraIOReal(CameraTransforms.frontLeftCameraConstants), drivetrain::addVisionMeasurement));
        break;
    }

    superstructure = new Superstructure(driver, drivetrain, intakeRollers);

    new Bindings(driver, operator, superstructure);

    autoFactory =
        new AutoFactory(
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
    // autoChooser.addRoutine("Left Auto - 4 Coral", autoRoutines::leftAutoRoutine);

    SmartDashboard.putData("AutoChooser", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    hood.updateInputs();
    flywheels.updateInputs();
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

  private void configureBindings() {
    driver.x().onTrue(Commands.runOnce(() -> hood.setWantedState(Hood.WantedState.STOPPED)));
    driver.x().onTrue(Commands.runOnce(() -> flywheels.setWantedState(Flywheels.WantedState.STOPPED)));

    driver.a().onTrue(Commands.runOnce(() -> flywheels.setWantedState(Flywheels.WantedState.SET_RPM, 1)));
    driver.b().onTrue(Commands.runOnce(() -> hood.setWantedState(Hood.WantedState.SET_POSITION, 4)));
  }
}
