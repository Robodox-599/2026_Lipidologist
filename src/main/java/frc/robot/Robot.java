// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;

public class Robot extends TimedRobot {
  private final CommandScheduler scheduler = CommandScheduler.getInstance();

  private final Feeder feeder;

  final CommandXboxController driver =
      new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
  final CommandXboxController operator = new CommandXboxController(Constants.ControllerConstants.kOperatorControllerPort);

@Override
  protected void loopFunc() {
    super.loopFunc();
  }

  public Robot() {
    feeder = new Feeder(new FeederIOSim());

    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withCaptureNt(true)
            .withNtPublish(true)
            .withCaptureConsole(true));

    switch (Constants.currentMode) {
      case REAL:
        break;
      default:
        break;
    }

    // new Bindings(driver, operator, superstructure);

    operator.rightBumper().onTrue(Commands.runOnce(() -> feeder.setWantedState(Feeder.WantedState.TRANSFERING_FUEL)));
    operator.rightTrigger().onTrue(Commands.runOnce(() -> feeder.setWantedState(Feeder.WantedState.REVERSE)));

    operator.x().onTrue(Commands.runOnce(() -> feeder.setWantedState(Feeder.WantedState.STOPPED)));
  }

  @Override
  public void robotPeriodic() {
    feeder.updateFeederInputs();
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
}
