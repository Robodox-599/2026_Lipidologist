package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoRoutines {
 private AutoFactory autoFactory;
  private Superstructure superstructureCommands;
  private CommandSwerveDrivetrain drivetrain;

  public AutoRoutines(
      AutoFactory autoFactory,
      Superstructure superstructureCommands,
      CommandSwerveDrivetrain drivetrain) {
    this.autoFactory = autoFactory;
    this.superstructureCommands = superstructureCommands;
    this.drivetrain = drivetrain;
  }

//   public AutoRoutine leftAutoRoutine() {
//     AutoRoutine routine = autoFactory.newRoutine("leftAuto");

//     AutoTrajectory LEFTtoIBack = routine.trajectory("LEFTtoIBack");
//     AutoTrajectory ItoHP = routine.trajectory("ItoHP");
//     AutoTrajectory IBacktoHP = routine.trajectory("IBacktoHP");
//     AutoTrajectory HPtoLBack = routine.trajectory("HPtoLBack");
//     AutoTrajectory LtoHP = routine.trajectory("LtoHP");
//     AutoTrajectory LBacktoHP = routine.trajectory("LBacktoHP");
//     AutoTrajectory HPtoKBack = routine.trajectory("HPtoKBack");
//     AutoTrajectory KtoHP = routine.trajectory("KtoHP");
//     AutoTrajectory KBacktoHP = routine.trajectory("KBacktoHP");
//     AutoTrajectory HPtoJBack = routine.trajectory("HPtoJBack");

//     routine
//         .active()
//         .onTrue(
//             Commands.sequence(
//                 LEFTtoIBack.resetOdometry(),
//                 superstructureCommands.setWantedSuperStateCommand(
//                     WantedSuperState.INTAKING_CORAL_STATION),
//                 LEFTtoIBack.cmd()));

//     LEFTtoIBack.done()
//         .onTrue(
//             Commands.either(
//                 superstructureCommands.setWantedSuperStateCommand(
//                     WantedSuperState.AUTO_SCORE_L4_LEFT_AUTO),
//                 IBacktoHP.cmd(),
//                 () -> !superstructureCommands.isCoralBranchScored()));

//     LEFTtoIBack.recentlyDone()
//         .and(() -> superstructureCommands.isCoralBranchScored())
//         .and(IBacktoHP.inactive())
//         .onTrue(
//             Commands.sequence(
//                 superstructureCommands.setWantedSuperStateCommand(
//                     WantedSuperState.INTAKING_CORAL_STATION),
//                 ItoHP.cmd()));

//     routine
//         .anyDone(ItoHP, IBacktoHP)
//         .onTrue(Commands.sequence(new WaitCommand(waitTimeAtHP), HPtoLBack.cmd()));

//     HPtoLBack.done()
//         .onTrue(
//             Commands.either(
//                 superstructureCommands.setWantedSuperStateCommand(
//                     WantedSuperState.AUTO_SCORE_L4_RIGHT_AUTO),
//                 LBacktoHP.cmd(),
//                 () -> !superstructureCommands.isCoralBranchScored()));

//     HPtoLBack.recentlyDone()
//         .and(() -> superstructureCommands.isCoralBranchScored())
//         .and(LBacktoHP.inactive())
//         .onTrue(
//             Commands.sequence(
//                 superstructureCommands.setWantedSuperStateCommand(
//                     WantedSuperState.INTAKING_CORAL_STATION),
//                 LtoHP.cmd()));

//     routine
//         .anyDone(LtoHP, LBacktoHP)
//         .onTrue(Commands.sequence(new WaitCommand(waitTimeAtHP), HPtoKBack.cmd()));

//     HPtoKBack.done()
//         .onTrue(
//             Commands.either(
//                 superstructureCommands.setWantedSuperStateCommand(
//                     WantedSuperState.AUTO_SCORE_L4_LEFT_AUTO),
//                 KBacktoHP.cmd(),
//                 () -> !superstructureCommands.isCoralBranchScored()));

//     HPtoKBack.recentlyDone()
//         .and(() -> superstructureCommands.isCoralBranchScored())
//         .and(KBacktoHP.inactive())
//         .onTrue(
//             Commands.sequence(
//                 superstructureCommands.setWantedSuperStateCommand(
//                     WantedSuperState.INTAKING_CORAL_STATION),
//                 KtoHP.cmd()));

//     routine
//         .anyDone(KtoHP, KBacktoHP)
//         .onTrue(Commands.sequence(new WaitCommand(waitTimeAtHP), HPtoJBack.cmd()));

//     HPtoJBack.recentlyDone()
//         .onTrue(
//             superstructureCommands.setWantedSuperStateCommand(
//                 WantedSuperState.AUTO_SCORE_L4_RIGHT_AUTO));

//     return routine;
//   }
}
