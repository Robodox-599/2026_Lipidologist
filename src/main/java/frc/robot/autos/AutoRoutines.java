package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain.WantedState;

public class AutoRoutines {
        private AutoFactory autoFactory;
        private Superstructure superstructureCommands;
        private CommandSwerveDrivetrain drivetrain;

        private static double shoot_without_agitation_secs = 1.0;
        private static double shoot_with_agitation_secs = 3.0;

        public AutoRoutines(
                        AutoFactory autoFactory,
                        Superstructure superstructureCommands,
                        CommandSwerveDrivetrain drivetrain) {
                this.autoFactory = autoFactory;
                this.superstructureCommands = superstructureCommands;
                this.drivetrain = drivetrain;
        }

        public AutoRoutine hubDoubleDouble(boolean isLeft) {
                AutoRoutine routine = autoFactory.newRoutine("hubDoubleDouble");

                AutoTrajectory trench_only_1 = isLeft ? routine.trajectory("trench_only_1")
                                : routine.trajectory("trench_only_1").mirrorY();
                AutoTrajectory trench_only_2_hub = isLeft ? routine.trajectory("trench_only_2_hub")
                                : routine.trajectory("trench_only_2_hub").mirrorY();
                AutoTrajectory trench_only_3 = isLeft ? routine.trajectory("trench_only_3")
                                : routine.trajectory("trench_only_3").mirrorY();

                routine.active().onTrue(Commands.parallel(disregardRequirements(trench_only_1.resetOdometry()),
                                Commands.runOnce(() -> drivetrain.setWantedState(WantedState.CHOREO_TRAJECTORY)),
                                superstructureCommands
                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                trench_only_1.cmd()));

                trench_only_1.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(shoot_without_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(shoot_with_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_only_2_hub.cmd()));

                trench_only_2_hub.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(shoot_without_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(shoot_with_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_only_3.cmd()));

                trench_only_3.done().onTrue(superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE));

                return routine;
        }

        public AutoRoutine midDoubleDouble(boolean isLeft) {
                AutoRoutine routine = autoFactory.newRoutine("midDoubleDouble");

                AutoTrajectory trench_only_1 = isLeft ? routine.trajectory("trench_only_1")
                                : routine.trajectory("trench_only_1").mirrorY();
                AutoTrajectory trench_only_2_mid = isLeft ? routine.trajectory("trench_only_2_mid")
                                : routine.trajectory("trench_only_2_mid").mirrorY();
                AutoTrajectory trench_only_3 = isLeft ? routine.trajectory("trench_only_3")
                                : routine.trajectory("trench_only_3").mirrorY();

                routine.active().onTrue(Commands.parallel(disregardRequirements(trench_only_1.resetOdometry()),
                                Commands.runOnce(() -> drivetrain.setWantedState(WantedState.CHOREO_TRAJECTORY)),
                                superstructureCommands
                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                trench_only_1.cmd()));

                trench_only_1.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(shoot_without_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(shoot_with_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_only_2_mid.cmd()));

                trench_only_2_mid.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(shoot_without_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(shoot_with_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_only_3.cmd()));

                trench_only_3.done().onTrue(superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE));

                return routine;
        }

        public AutoRoutine hubAnimalStyle(boolean isLeft) {
                AutoRoutine routine = autoFactory.newRoutine("hubAnimalStyle");

                AutoTrajectory trench_and_bump_1 = isLeft ? routine.trajectory("trench_and_bump_1")
                                : routine.trajectory("trench_and_bump_1").mirrorY();
                AutoTrajectory trench_and_bump_2_hub = isLeft ? routine.trajectory("trench_and_bump_2_hub")
                                : routine.trajectory("trench_and_bump_2_hub").mirrorY();
                AutoTrajectory trench_only_3 = isLeft ? routine.trajectory("trench_only_3")
                                : routine.trajectory("trench_only_3").mirrorY();

                routine.active().onTrue(Commands.parallel(disregardRequirements(trench_and_bump_1.resetOdometry()),
                                Commands.runOnce(() -> drivetrain.setWantedState(WantedState.CHOREO_TRAJECTORY)),
                                superstructureCommands
                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                trench_and_bump_1.cmd()));

                trench_and_bump_1.atTime("lift_intake").onTrue(
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.LIFT_INTAKE_AUTO));

                trench_and_bump_1.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(shoot_without_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(shoot_with_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_and_bump_2_hub.cmd()));

                trench_and_bump_2_hub.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(shoot_without_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(shoot_with_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_only_3.cmd()));

                trench_only_3.done().onTrue(superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE));

                return routine;
        }

        public AutoRoutine midAnimalStyle(boolean isLeft) {
                AutoRoutine routine = autoFactory.newRoutine("midAnimalStyle");

                AutoTrajectory trench_and_bump_1 = isLeft ? routine.trajectory("trench_and_bump_1")
                                : routine.trajectory("trench_and_bump_1").mirrorY();
                AutoTrajectory trench_and_bump_2_mid = isLeft ? routine.trajectory("trench_and_bump_2_mid")
                                : routine.trajectory("trench_and_bump_2_mid").mirrorY();
                AutoTrajectory trench_and_bump_3 = isLeft ? routine.trajectory("trench_and_bump_3")
                                : routine.trajectory("trench_and_bump_3").mirrorY();

                routine.active().onTrue(Commands.parallel(disregardRequirements(trench_and_bump_1.resetOdometry()),
                                Commands.runOnce(() -> drivetrain.setWantedState(WantedState.CHOREO_TRAJECTORY)),
                                superstructureCommands
                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                trench_and_bump_1.cmd()));

                trench_and_bump_1.atTime("lift_intake").onTrue(
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.LIFT_INTAKE_AUTO));

                trench_and_bump_1.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(shoot_without_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(shoot_with_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_and_bump_2_mid.cmd()));

                trench_and_bump_2_mid.atTime("lift_intake").onTrue(
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.LIFT_INTAKE_AUTO));

                trench_and_bump_2_mid.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(shoot_without_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(shoot_with_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_and_bump_3.cmd()));

                trench_and_bump_3.done()
                                .onTrue(superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE));

                return routine;
        }

        public AutoRoutine safeDoubleDouble(boolean isLeft) {
                AutoRoutine routine = autoFactory.newRoutine("safeDoubleDouble");

                AutoTrajectory trench_only_1 = isLeft ? routine.trajectory("trench_only_1")
                                : routine.trajectory("trench_only_1").mirrorY();
                AutoTrajectory trench_only_2_safe = isLeft ? routine.trajectory("trench_only_2_safe")
                                : routine.trajectory("trench_only_2_safe").mirrorY();
                AutoTrajectory trench_only_3 = isLeft ? routine.trajectory("trench_only_3")
                                : routine.trajectory("trench_only_3").mirrorY();

                routine.active().onTrue(Commands.parallel(disregardRequirements(trench_only_1.resetOdometry()),
                                Commands.runOnce(() -> drivetrain.setWantedState(WantedState.CHOREO_TRAJECTORY)),
                                superstructureCommands
                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                trench_only_1.cmd()));

                trench_only_1.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(shoot_without_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(shoot_with_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_only_2_safe.cmd()));

                trench_only_2_safe.done()
                                .onTrue(Commands.sequence(superstructureCommands
                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(shoot_without_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(shoot_with_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_only_3.cmd()));

                trench_only_3.done().onTrue(superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE));

                return routine;
        }

        public AutoRoutine safeAnimalStyle(boolean isLeft) {
                AutoRoutine routine = autoFactory.newRoutine("safeAnimalStyle");

                AutoTrajectory trench_and_bump_1 = isLeft ? routine.trajectory("trench_and_bump_1")
                                : routine.trajectory("trench_and_bump_1").mirrorY();
                AutoTrajectory trench_and_bump_2_safe = isLeft ? routine.trajectory("trench_and_bump_2_safe")
                                : routine.trajectory("trench_and_bump_2_safe").mirrorY();
                AutoTrajectory trench_and_bump_3 = isLeft ? routine.trajectory("trench_and_bump_3")
                                : routine.trajectory("trench_and_bump_3").mirrorY();

                routine.active().onTrue(Commands.parallel(disregardRequirements(trench_and_bump_1.resetOdometry()),
                                Commands.runOnce(() -> drivetrain.setWantedState(WantedState.CHOREO_TRAJECTORY)),
                                superstructureCommands
                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                trench_and_bump_1.cmd()));

                trench_and_bump_1.atTime("lift_intake").onTrue(
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.LIFT_INTAKE_AUTO));

                trench_and_bump_1.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(shoot_without_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(shoot_with_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_and_bump_2_safe.cmd()));

                trench_and_bump_2_safe.atTime("lift_intake").onTrue(
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.LIFT_INTAKE_AUTO));

                trench_and_bump_2_safe.done()
                                .onTrue(Commands.sequence(superstructureCommands
                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(shoot_without_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(shoot_with_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                trench_and_bump_3.cmd()));

                trench_and_bump_3.done()
                                .onTrue(superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE));

                return routine;
        }

        public AutoRoutine safeNeapolitanMilkshake(boolean isLeft) {
                AutoRoutine routine = autoFactory.newRoutine("safeNeapolitanMilkshake");

                AutoTrajectory bump_only_1_safe = isLeft ? routine.trajectory("bump_only_1_safe")
                                : routine.trajectory("bump_only_1_safe").mirrorY();

                routine.active().onTrue(Commands.sequence(
                                Commands.parallel(
                                                disregardRequirements(bump_only_1_safe.resetOdometry()),
                                                Commands.runOnce(() -> drivetrain
                                                                .setWantedState(WantedState.CHOREO_TRAJECTORY)),
                                                superstructureCommands.setWantedSuperStateCommand(
                                                                WantedSuperState.LIFT_INTAKE_AUTO)),
                                new WaitCommand(1.5),
                                bump_only_1_safe.cmd()));

                bump_only_1_safe.atTime("deploy_intake").onTrue(
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO));

                bump_only_1_safe.atTime("lift_intake").onTrue(
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.LIFT_INTAKE_AUTO));

                bump_only_1_safe.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(shoot_without_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(shoot_with_agitation_secs)));

                return routine;
        }
        
        public AutoRoutine greedyNeapolitanMilkshake(boolean isLeft) {
                AutoRoutine routine = autoFactory.newRoutine("greedyNeapolitanMilkshake");

                AutoTrajectory bump_only_1_greedy = isLeft ? routine.trajectory("bump_only_1_greedy")
                                : routine.trajectory("bump_only_1_greedy").mirrorY();

                routine.active().onTrue(Commands.sequence(
                                Commands.parallel(
                                                disregardRequirements(bump_only_1_greedy.resetOdometry()),
                                                Commands.runOnce(() -> drivetrain
                                                                .setWantedState(WantedState.CHOREO_TRAJECTORY)),
                                                superstructureCommands.setWantedSuperStateCommand(
                                                                WantedSuperState.LIFT_INTAKE_AUTO)),
                                new WaitCommand(1.5),
                                bump_only_1_greedy.cmd()));

                bump_only_1_greedy.atTime("deploy_intake").onTrue(
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO));

                bump_only_1_greedy.atTime("lift_intake").onTrue(
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.LIFT_INTAKE_AUTO));

                bump_only_1_greedy.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(shoot_without_agitation_secs),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(shoot_with_agitation_secs)));

                return routine;
        }

        // public AutoRoutine rightDoubleSteakBurrito() {
        // AutoRoutine routine = autoFactory.newRoutine("rightDoubleSteakBurrito");

        // AutoTrajectory trench_only_1 = routine.trajectory("trench_only_1").mirrorY();
        // AutoTrajectory trench_only_2_hub =
        // routine.trajectory("trench_only_2_hub").mirrorY();

        // routine.active().onTrue(Commands.sequence(trench_only_1.resetOdometry(),
        // Commands.parallel(
        // superstructureCommands
        // .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
        // trench_only_1.cmd())));

        // trench_only_1.done()
        // .onTrue(Commands.sequence(
        // superstructureCommands
        // .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
        // new WaitCommand(1.0),
        // superstructureCommands
        // .setWantedSuperStateCommand(
        // WantedSuperState.SHOOT_HUB_AND_AGITATE),
        // new WaitCommand(3.5),
        // superstructureCommands
        // .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
        // trench_only_2_hub.cmd()));

        // trench_only_2_hub.done()
        // .onTrue(Commands.sequence(
        // superstructureCommands
        // .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
        // new WaitCommand(1),
        // superstructureCommands
        // .setWantedSuperStateCommand(
        // WantedSuperState.SHOOT_HUB_AND_AGITATE)));

        // return routine;
        // }

        // public AutoRoutine rightDoubleSteakBowl() {
        // AutoRoutine routine = autoFactory.newRoutine("rightDoubleSteakBowl");

        // AutoTrajectory trench_only_1 = routine.trajectory("trench_only_1").mirrorY();
        // AutoTrajectory trench_and_bump_2_mid =
        // routine.trajectory("trench_only_2_mid").mirrorY();

        // routine.active().onTrue(Commands.sequence(trench_only_1.resetOdometry(),
        // Commands.parallel(
        // superstructureCommands
        // .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
        // trench_only_1.cmd())));

        // trench_only_1.done()
        // .onTrue(Commands.sequence(
        // superstructureCommands
        // .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
        // new WaitCommand(1.0),
        // superstructureCommands
        // .setWantedSuperStateCommand(
        // WantedSuperState.SHOOT_HUB_AND_AGITATE),
        // new WaitCommand(3.5),
        // superstructureCommands
        // .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
        // trench_and_bump_2_mid.cmd()));

        // trench_and_bump_2_mid.done()
        // .onTrue(Commands.sequence(
        // superstructureCommands
        // .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
        // new WaitCommand(1),
        // superstructureCommands
        // .setWantedSuperStateCommand(
        // WantedSuperState.SHOOT_HUB_AND_AGITATE)));

        // return routine;
        // }

        // public AutoRoutine rightChickenAndSteakBurrito() {
        // AutoRoutine routine = autoFactory.newRoutine("rightChickenAndSteakBurrito");

        // AutoTrajectory trench_and_bump_1 =
        // routine.trajectory("trench_and_bump_1").mirrorY();
        // AutoTrajectory trench_and_bump_2_hub =
        // routine.trajectory("trench_and_bump_2_hub").mirrorY();

        // routine.active().onTrue(Commands.sequence(trench_and_bump_1.resetOdometry(),
        // Commands.parallel(
        // superstructureCommands
        // .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
        // trench_and_bump_1.cmd())));

        // trench_and_bump_1.done()
        // .onTrue(Commands.sequence(
        // superstructureCommands
        // .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
        // new WaitCommand(1.0),
        // superstructureCommands
        // .setWantedSuperStateCommand(
        // WantedSuperState.SHOOT_HUB_AND_AGITATE),
        // new WaitCommand(3.5),
        // superstructureCommands
        // .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
        // trench_and_bump_2_hub.cmd()));

        // trench_and_bump_2_hub.done()
        // .onTrue(Commands.sequence(
        // superstructureCommands
        // .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
        // new WaitCommand(1),
        // superstructureCommands
        // .setWantedSuperStateCommand(
        // WantedSuperState.SHOOT_HUB_AND_AGITATE)));

        // return routine;
        // }

        // public AutoRoutine rightChickenAndSteakBowl() {
        // AutoRoutine routine = autoFactory.newRoutine("rightChickenAndSteakBowl");

        // AutoTrajectory trench_and_bump_1 =
        // routine.trajectory("trench_and_bump_1").mirrorY();
        // AutoTrajectory trench_and_bump_2_mid =
        // routine.trajectory("trench_and_bump_2_mid").mirrorY();

        // routine.active().onTrue(Commands.sequence(trench_and_bump_1.resetOdometry(),
        // Commands.parallel(
        // superstructureCommands
        // .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
        // trench_and_bump_1.cmd())));

        // trench_and_bump_1.done()
        // .onTrue(Commands.sequence(
        // superstructureCommands
        // .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
        // new WaitCommand(1.0),
        // superstructureCommands
        // .setWantedSuperStateCommand(
        // WantedSuperState.SHOOT_HUB_AND_AGITATE),
        // new WaitCommand(3.5),
        // superstructureCommands
        // .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
        // trench_and_bump_2_mid.cmd()));

        // trench_and_bump_2_mid.done()
        // .onTrue(Commands.sequence(
        // superstructureCommands
        // .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
        // new WaitCommand(1),
        // superstructureCommands
        // .setWantedSuperStateCommand(
        // WantedSuperState.SHOOT_HUB_AND_AGITATE)));

        // return routine;
        // }

        // public AutoRoutine leftHamburgerWithOnions() {
        // AutoRoutine routine = autoFactory.newRoutine("LeftHamburgerWithOnions");

        // AutoTrajectory bump_to_depot = routine.trajectory("bump_to_depot");

        // routine.active().onTrue(Commands.sequence(bump_to_depot.resetOdometry(),
        // superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
        // bump_to_depot.cmd()));

        // bump_to_depot.done()
        // .onTrue(
        // // superstructureCommands
        // // .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
        // // new WaitCommand(1),
        // superstructureCommands
        // .setWantedSuperStateCommand(
        // WantedSuperState.SHOOT_HUB_AND_AGITATE));

        // return routine;
        // }

        public Command disregardRequirements(Command cmd) {
                return new FunctionalCommand(
                                cmd::initialize,
                                cmd::execute,
                                cmd::end,
                                cmd::isFinished
                // no requirements passed = empty by default
                );
        }
}
