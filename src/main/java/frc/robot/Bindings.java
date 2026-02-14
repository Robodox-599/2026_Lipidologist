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

public class Bindings extends SubsystemBase {

  private final Superstructure superstructure;

  public Bindings(
      CommandXboxController driver, Superstructure superstructure) {
    this.superstructure = superstructure;

    driver.y().onTrue(superstructure.zeroPoseCommand());

    // driver.rightTrigger().whileTrue(new RepeatCommand(setShootingStateCommand())).onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.IDLE));

    // driver.povRight().and(driver.rightTrigger().negate()).whileTrue(new RepeatCommand(setPrepareShootingStateCommand())).onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.IDLE));

    driver.povRight().whileTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB)).onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.IDLE));

    new Trigger(() -> HubShiftUtil.isHubLookaheadActive(3)).onTrue(rumbleDriverSwapping(driver, 0.5, 3));
    
    // driver.a().onTrue(rumbleDriverContinuous(driver, 3));
    // driver.b().onTrue(rumbleDriverSwapping(driver, 0.5, 3));
  }

  public Command setShootingStateCommand() {
    return Commands.defer(
        () -> superstructure.setWantedSuperStateCommand(returnShootingState()),
        Set.of(superstructure));
  }

  public Command setPrepareShootingStateCommand() {
    return Commands.defer(
        () -> superstructure.setWantedSuperStateCommand(returnPrepareShootingState()),
        Set.of(superstructure));
  }

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

  public WantedSuperState returnPrepareShootingState() {
    if (AllianceFlipUtil.shouldFlip()) { // if red
      if (superstructure.getPose().getX() > AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone)) {
        return WantedSuperState.PREPARE_HUB_SHOT;
      } else {
        return WantedSuperState.PREPARE_ALLIANCE_ZONE_SHOT;
      }
    } else {
      if (superstructure.getPose().getX() < AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone)) {
        return WantedSuperState.PREPARE_HUB_SHOT;
      } else {
        return WantedSuperState.PREPARE_ALLIANCE_ZONE_SHOT;
      }
    }
  }

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
