// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.constants.TunerConstants;
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
  //     Constants.ControllerConstants.kOperatorControllerPort);
  // final CommandSwerveDrivetrain drivetrain;

  private final IntakeRollers intakeRollers;
  private final IntakeWrist intakeWrist;

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
        // drivetrain = TunerConstants.createDrivetrain(driver);
        intakeRollers = new IntakeRollers(new IntakeRollersIOTalonFX());
        intakeWrist = new IntakeWrist(new IntakeWristIOTalonFX());
        break;
      case SIM:
        // drivetrain = TunerConstants.createDrivetrain(driver);
        intakeRollers = new IntakeRollers(new IntakeRollersIOSim());
        intakeWrist = new IntakeWrist(new IntakeWristIOSim());
        break;
      default:
        // drivetrain = TunerConstants.createDrivetrain(driver);
        intakeRollers = new IntakeRollers(new IntakeRollersIOSim());
        intakeWrist = new IntakeWrist(new IntakeWristIOSim());
        break;
    }

    // new Bindings(driver, operator, superstructure);

       driver.x().onTrue(Commands.runOnce(() -> intakeWrist.setWantedState(IntakeWrist.WristWantedState.INTAKE_FUEL)));
       driver.y().onTrue(Commands.runOnce(() -> intakeWrist.setWantedState(IntakeWrist.WristWantedState.STOW)));
       driver.b().onTrue(Commands.runOnce(() -> intakeWrist.setWantedState(IntakeWrist.WristWantedState.AGITATE_FUEL)));
  }

  @Override
  public void robotPeriodic() {
    intakeRollers.updateInputs();
    intakeWrist.updateInputs();
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
