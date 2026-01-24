package frc.robot;

import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIOTalonFX;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollersIOTalonFX;

public class Robot extends TimedRobot {
  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  private final Hood hood;
  private final Flywheels flywheels;

  final CommandXboxController driver =
      new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
  final CommandXboxController operator = new CommandXboxController(Constants.ControllerConstants.kOperatorControllerPort);
  final CommandSwerveDrivetrain drivetrain;
  final IntakeRollers intakeRollers;
  final Superstructure superstructure;

  // final AutoFactory autoFactory;

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

    hood = new Hood(new HoodIOTalonFX());
    flywheels = new Flywheels(new FlywheelsIOTalonFX());

    switch (Constants.currentMode) {
      case REAL:
        drivetrain = TunerConstants.createDrivetrain(driver);
        intakeRollers = new IntakeRollers(new IntakeRollersIOTalonFX());
        break;
      default:
        drivetrain = TunerConstants.createDrivetrain(driver);
        intakeRollers = new IntakeRollers(new IntakeRollersIOTalonFX());
        break;
    }

    superstructure = new Superstructure(driver, drivetrain, intakeRollers);

    new Bindings(driver, operator, superstructure);
  }

  @Override
  public void robotPeriodic() {
    hood.updateInputs();
    flywheels.updateInputs();
    scheduler.run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    scheduler.cancelAll();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    scheduler.cancelAll();
    
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    scheduler.cancelAll();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

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
