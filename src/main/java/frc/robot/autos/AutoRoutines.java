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

        public AutoRoutine leftDoubleSteakBurrito() {
                AutoRoutine routine = autoFactory.newRoutine("leftDoubleSteakBurrito");

                AutoTrajectory trench_only_1 = routine.trajectory("trench_only_1");
                AutoTrajectory trench_only_2_hub = routine.trajectory("trench_only_2_hub");

                routine.active().onTrue(Commands.sequence(trench_only_1.resetOdometry(),
                                Commands.parallel(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_only_1.cmd())));

                trench_only_1.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1.0),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(3.5),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_only_2_hub.cmd()));

                trench_only_2_hub.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE)));

                return routine;
        }

        public AutoRoutine leftDoubleSteakBowl() {
                AutoRoutine routine = autoFactory.newRoutine("leftDoubleSteakBowl");

                AutoTrajectory trench_only_1 = routine.trajectory("trench_only_1");
                AutoTrajectory trench_and_bump_2_mid = routine.trajectory("trench_only_1_mid");

                routine.active().onTrue(Commands.sequence(trench_only_1.resetOdometry(),
                                Commands.parallel(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_only_1.cmd())));

                trench_only_1.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1.0),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(3.5),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_and_bump_2_mid.cmd()));

                trench_and_bump_2_mid.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE)));

                return routine;
        }

        public AutoRoutine leftChickenAndSteakBurrito() {
                AutoRoutine routine = autoFactory.newRoutine("leftChickenAndSteakBurrito");

                AutoTrajectory trench_and_bump_1 = routine.trajectory("trench_and_bump_1");
                AutoTrajectory trench_and_bump_2_hub = routine.trajectory("trench_and_bump_2_hub");

                routine.active().onTrue(Commands.sequence(trench_and_bump_1.resetOdometry(),
                                Commands.parallel(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_and_bump_1.cmd())));

                trench_and_bump_1.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1.0),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(3.5),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_and_bump_2_hub.cmd()));

                trench_and_bump_2_hub.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE)));

                return routine;
        }

        public AutoRoutine leftChickenAndSteakBowl() {
                AutoRoutine routine = autoFactory.newRoutine("leftChickenAndSteakBowl");

                AutoTrajectory trench_and_bump_1 = routine.trajectory("trench_and_bump_1");
                AutoTrajectory trench_and_bump_2_mid = routine.trajectory("trench_and_bump_2_mid");

                routine.active().onTrue(Commands.sequence(trench_and_bump_1.resetOdometry(),
                                Commands.parallel(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_and_bump_1.cmd())));

                trench_and_bump_1.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1.0),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(3.5),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_and_bump_2_mid.cmd()));

                trench_and_bump_2_mid.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE)));

                return routine;
        }

        public AutoRoutine rightDoubleSteakBurrito() {
                AutoRoutine routine = autoFactory.newRoutine("rightDoubleSteakBurrito");

                AutoTrajectory trench_only_1 = routine.trajectory("trench_only_1").mirrorY();
                AutoTrajectory trench_only_2_hub = routine.trajectory("trench_only_2_hub").mirrorY();

                routine.active().onTrue(Commands.sequence(trench_only_1.resetOdometry(),
                                Commands.parallel(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_only_1.cmd())));

                trench_only_1.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1.0),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(3.5),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_only_2_hub.cmd()));

                trench_only_2_hub.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE)));

                return routine;
        }

        public AutoRoutine rightDoubleSteakBowl() {
                AutoRoutine routine = autoFactory.newRoutine("rightDoubleSteakBowl");

                AutoTrajectory trench_only_1 = routine.trajectory("trench_only_1").mirrorY();
                AutoTrajectory trench_and_bump_2_mid = routine.trajectory("trench_only_2_mid").mirrorY();

                routine.active().onTrue(Commands.sequence(trench_only_1.resetOdometry(),
                                Commands.parallel(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_only_1.cmd())));

                trench_only_1.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1.0),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(3.5),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_and_bump_2_mid.cmd()));

                trench_and_bump_2_mid.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE)));

                return routine;
        }

        public AutoRoutine rightChickenAndSteakBurrito() {
                AutoRoutine routine = autoFactory.newRoutine("rightChickenAndSteakBurrito");

                AutoTrajectory trench_and_bump_1 = routine.trajectory("trench_and_bump_1").mirrorY();
                AutoTrajectory trench_and_bump_2_hub = routine.trajectory("trench_and_bump_2_hub").mirrorY();

                routine.active().onTrue(Commands.sequence(trench_and_bump_1.resetOdometry(),
                                Commands.parallel(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_and_bump_1.cmd())));

                trench_and_bump_1.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1.0),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(3.5),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_and_bump_2_hub.cmd()));

                trench_and_bump_2_hub.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE)));

                return routine;
        }

        public AutoRoutine rightChickenAndSteakBowl() {
                AutoRoutine routine = autoFactory.newRoutine("rightChickenAndSteakBowl");

                AutoTrajectory trench_and_bump_1 = routine.trajectory("trench_and_bump_1").mirrorY();
                AutoTrajectory trench_and_bump_2_mid = routine.trajectory("trench_and_bump_2_mid").mirrorY();

                routine.active().onTrue(Commands.sequence(trench_and_bump_1.resetOdometry(),
                                Commands.parallel(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_and_bump_1.cmd())));

                trench_and_bump_1.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1.0),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(3.5),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_and_bump_2_mid.cmd()));

                trench_and_bump_2_mid.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE)));

                return routine;
        }

        public AutoRoutine leftHamburgerWithOnions() {
                AutoRoutine routine = autoFactory.newRoutine("LeftHamburgerWithOnions");

                AutoTrajectory bump_to_depot = routine.trajectory("bump_to_depot");

                routine.active().onTrue(Commands.sequence(bump_to_depot.resetOdometry(),
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                bump_to_depot.cmd()));

                bump_to_depot.done()
                                .onTrue(
                                                // superstructureCommands
                                                // .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                // new WaitCommand(1),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE));

                return routine;
        }
}
