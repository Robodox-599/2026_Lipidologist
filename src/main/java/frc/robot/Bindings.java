package frc.robot;

import edu.wpi.first.wpilibj2.command.StartEndCommand;

import java.util.Set;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.HubShiftUtil;

public class Bindings {

  private final Superstructure superstructure;

  public Bindings(
      CommandXboxController driver, Superstructure superstructure) {
    this.superstructure = superstructure;

    driver.y().onTrue(superstructure.zeroPoseCommand());

    // AUTOMATICALLY SHOOT WHEN READY TO EITHER HUB OR ALLIANCE ZONE
    driver.rightTrigger().and(driver.leftTrigger().negate())
        .whileTrue(new RepeatCommand(setShootingStateCommand()))
        .onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.IDLE));

    // AUTOMATICALLY SHOOT WHILE AGITATING WHEN READY TO EITHER HUB OR ALLIANCE ZONE
    driver.leftTrigger().whileTrue(new RepeatCommand(setAgitatingShootingStateCommand())).onFalse(Commands.either(
        Commands.none(), superstructure.setWantedSuperStateCommand(WantedSuperState.IDLE), driver.rightTrigger()));

    // SHOOT MANUALLY (IN FRONT OF HUB)
    driver.a().and(driver.leftTrigger().negate()).and(driver.rightTrigger().negate()).and(driver.rightBumper().negate())
        .whileTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB_MANUAL))
        .onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.IDLE));

    // OUTAKE FUEL
    driver.rightBumper().and(driver.leftTrigger().negate()).and(driver.rightTrigger().negate())
        .whileTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.OUTAKE))
        .onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.IDLE));

    //CLEAN SYSTEMs
    // driver.x()
    //     .whileTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.CLEAN))
    //     .onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.STOP));
    
    new Trigger(() -> HubShiftUtil.isHubActiveSoon(5)).onTrue(rumbleDriverSwapping(driver, 0.5, 5));

    // // AUTOMATICALLY CLIMB
    // driver.a().onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.CLIMB)).onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.IDLE));

    // driver.rightTrigger().onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.TESTING))
    // .onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.IDLE));

    driver.b()
      .onTrue(Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.TUNE_SHOT_DATA_SHOOT)))
      .onFalse(Commands.runOnce(() -> superstructure.setWantedSuperState(WantedSuperState.TUNE_SHOT_DATA_IDLE)));
  }

  public Command setShootingStateCommand() {
    return Commands.defer(
        () -> superstructure.setWantedSuperStateCommand(returnShootingState()),
        Set.of(superstructure));
  }

  public Command setAgitatingShootingStateCommand() {
    return Commands.defer(
        () -> superstructure.setWantedSuperStateCommand(returnAgitatingShootingState()),
        Set.of(superstructure));
  }

  // public Command setPrepareShootingStateCommand() {
  // return Commands.defer(
  // () ->
  // superstructure.setWantedSuperStateCommand(returnPrepareShootingState()),
  // Set.of(superstructure));
  // }

  public WantedSuperState returnShootingState() {
    if (AllianceFlipUtil.shouldFlip()) { // if red
      if (superstructure.getPose().getX() > AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone)) {
        return WantedSuperState.SHOOT_HUB;
      } else {
        return WantedSuperState.SHOOT_ALLIANCE_ZONE;
      }
    } else {
      if (superstructure.getPose().getX() < AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone)) {
        return WantedSuperState.SHOOT_HUB;
      } else {
        return WantedSuperState.SHOOT_ALLIANCE_ZONE;
      }
    }
  }

  public WantedSuperState returnAgitatingShootingState() {
    if (AllianceFlipUtil.shouldFlip()) { // if red
      if (superstructure.getPose().getX() > AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone)) {
        return WantedSuperState.SHOOT_HUB_AND_AGITATE;
      } else {
        return WantedSuperState.SHOOT_ALLIANCE_ZONE_AND_AGITATE;
      }
    } else {
      if (superstructure.getPose().getX() < AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone)) {
        return WantedSuperState.SHOOT_HUB_AND_AGITATE;
      } else {
        return WantedSuperState.SHOOT_ALLIANCE_ZONE_AND_AGITATE;
      }
    }
  }

  // public WantedSuperState returnPrepareShootingState() {
  // if (AllianceFlipUtil.shouldFlip()) { // if red
  // if (superstructure.getPose().getX() >
  // AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone)) {
  // return WantedSuperState.PREPARE_HUB_SHOT;
  // } else {
  // return WantedSuperState.PREPARE_ALLIANCE_ZONE_SHOT;
  // }
  // } else {
  // if (superstructure.getPose().getX() <
  // AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone)) {
  // return WantedSuperState.PREPARE_HUB_SHOT;
  // } else {
  // return WantedSuperState.PREPARE_ALLIANCE_ZONE_SHOT;
  // }
  // }
  // }

  public Command rumbleDriverContinuous(CommandXboxController driver, double totalTime) {
    return new StartEndCommand(
        () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
        () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
        .withTimeout(totalTime);
  }

  public Command rumbleDriverSwapping(CommandXboxController driver, double swapTime, double totalTime) {
    return Commands.sequence(
        Commands.runOnce(() -> driver.getHID().setRumble(RumbleType.kLeftRumble, 1)),
        Commands.waitSeconds(swapTime),
        Commands.runOnce(() -> driver.getHID().setRumble(RumbleType.kLeftRumble, 0)),
        Commands.runOnce(() -> driver.getHID().setRumble(RumbleType.kRightRumble, 1)),
        Commands.waitSeconds(swapTime),
        Commands.runOnce(() -> driver.getHID().setRumble(RumbleType.kRightRumble, 0)))
        .repeatedly().withTimeout(totalTime)
        .finallyDo(() -> driver.getHID().setRumble(RumbleType.kBothRumble, 0));
  }

  // public Command rumbleControllers(CommandXboxController driver,
  // CommandXboxController operator) {
  // return new StartEndCommand(
  // () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
  // () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
  // .alongWith(
  // new StartEndCommand(
  // () -> operator.getHID().setRumble(RumbleType.kBothRumble, 1),
  // () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0)))
  // .withTimeout(0.25);
  // }

  // public Command rumbleOperator(CommandXboxController operator) {
  // return new StartEndCommand(
  // () -> operator.getHID().setRumble(RumbleType.kBothRumble, 1),
  // () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0))
  // .withTimeout(0.1);
  // }
}
