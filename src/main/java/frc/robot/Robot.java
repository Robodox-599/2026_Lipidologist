package frc.robot;

import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIOSim;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIOTalonFX;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;

public class Robot extends TimedRobot {
  public static final RobotType ROBOT_TYPE = Robot.isReal() ? RobotType.REAL : RobotType.SIM;
  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  private final Hood hood;
  private final Flywheels flywheels;

  final CommandXboxController driver =
      new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
  final CommandXboxController operator = new CommandXboxController(Constants.ControllerConstants.kOperatorControllerPort);
  // final Superstructure superstructure = new Superstructure();

  // final AutoFactory autoFactory;

  public enum RobotType {
    REAL,
    SIM
  }

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

    hood = new Hood(ROBOT_TYPE != RobotType.SIM ? new HoodIOTalonFX() : new HoodIOSim());
    flywheels = new Flywheels(ROBOT_TYPE != RobotType.SIM ? new FlywheelsIOTalonFX() : new FlywheelsIOSim());

    switch (Constants.currentMode) {
      case REAL:
        break;
      default:
        break;
    }
    configureBindings();
    // new Bindings(driver, operator, superstructure);
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

    driver.leftTrigger().onTrue(Commands.runOnce(() -> flywheels.setVoltage(1)));
    driver.rightTrigger().onTrue(Commands.runOnce(() -> hood.setVoltage(1)));
   }
}
