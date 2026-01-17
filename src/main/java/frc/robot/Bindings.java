package frc.robot;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;

public class Bindings extends SubsystemBase {
    
  private final Superstructure superstructure;

  public Bindings(
      CommandXboxController driver, CommandXboxController operatorXboxController, Superstructure superstructure) {
        this.superstructure = superstructure;

        driver.y().onTrue(superstructure.zeroGyroCommand());

        driver.rightTrigger().onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.IDLE));

        driver.rightTrigger().onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.STOP));

        
      }

      public Command rumbleControllers(CommandXboxController driver, CommandXboxController operator) {
        return new StartEndCommand(
                () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
                () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
            .alongWith(
                new StartEndCommand(
                    () -> operator.getHID().setRumble(RumbleType.kBothRumble, 1),
                    () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0)))
            .withTimeout(0.25);
      }
    
      public Command rumbleOperator(CommandXboxController operator) {
        return new StartEndCommand(
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 1),
                () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0))
            .withTimeout(0.1);
      }
    
      public Command rumbleDriver(CommandXboxController driver) {
        return new StartEndCommand(
                () -> driver.getHID().setRumble(RumbleType.kBothRumble, 1),
                () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0))
            .withTimeout(0.2);
      }
}
