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

        public AutoRoutine leftDoubleDouble() {
                AutoRoutine routine = autoFactory.newRoutine("leftDoubleDouble");

                AutoTrajectory LTRENCHtoLMID = routine.trajectory("LTRENCHtoLMID");
                AutoTrajectory LMIDtoLTRENCH = routine.trajectory("LMIDtoLTRENCH");
                AutoTrajectory LTRENCHtoHUBSWEEP = routine.trajectory("LTRENCHtoHUBSWEEP");

                routine.active().onTrue(Commands.sequence(LTRENCHtoLMID.resetOdometry(),
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                LTRENCHtoLMID.cmd()));

                LTRENCHtoLMID.done().onTrue(LMIDtoLTRENCH.cmd());

                LMIDtoLTRENCH.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(2.5),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                LTRENCHtoHUBSWEEP.cmd()));

                LTRENCHtoHUBSWEEP.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE)));

                return routine;
        }

        public AutoRoutine leftDoubleDoubleAnimalStyle() {
                AutoRoutine routine = autoFactory.newRoutine("leftDoubleDoubleAnimalStyle");

                AutoTrajectory LTRENCHtoLMID = routine.trajectory("LTRENCHtoLMID");
                AutoTrajectory LMIDtoLBUMP = routine.trajectory("LMIDtoLBUMP");
                AutoTrajectory LBUMPtoLTRENCH = routine.trajectory("LBUMPtoLTRENCH");
                AutoTrajectory LTRENCHtoHUBSWEEP = routine.trajectory("LTRENCHtoHUBSWEEP");

                routine.active().onTrue(Commands.sequence(LTRENCHtoLMID.resetOdometry(),
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                LTRENCHtoLMID.cmd()));

                LTRENCHtoLMID.done().onTrue(LMIDtoLBUMP.cmd());

                LMIDtoLBUMP.done()
                                .onTrue(Commands.sequence(LBUMPtoLTRENCH.cmd(),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SOTM_HUB_AUTO)));

                LBUMPtoLTRENCH.done().onTrue(Commands.sequence(
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                LTRENCHtoHUBSWEEP.cmd()));

                LTRENCHtoHUBSWEEP.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE)));

                return routine;
        }

        public AutoRoutine rightDoubleDouble() {
                AutoRoutine routine = autoFactory.newRoutine("rightDoubleDouble");

                AutoTrajectory RTRENCHtoRMID = routine.trajectory("RTRENCHtoRMID");
                AutoTrajectory RMIDtoRTRENCH = routine.trajectory("RMIDtoRTRENCH");
                AutoTrajectory RTRENCHtoHUBSWEEP = routine.trajectory("RTRENCHtoHUBSWEEP");

                routine.active().onTrue(Commands.sequence(RTRENCHtoRMID.resetOdometry(),
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                RTRENCHtoRMID.cmd()));

                RTRENCHtoRMID.done().onTrue(RMIDtoRTRENCH.cmd());

                RMIDtoRTRENCH.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(2.5),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                RTRENCHtoHUBSWEEP.cmd()));

                RTRENCHtoHUBSWEEP.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE)));

                return routine;
        }
        
        public AutoRoutine rightDoubleDoubleAnimalStyle() {
                AutoRoutine routine = autoFactory.newRoutine("rightDoubleDoubleAnimalStyle");

                AutoTrajectory RTRENCHtoRMID = routine.trajectory("RTRENCHtoRMID");
                AutoTrajectory RMIDtoRBUMP = routine.trajectory("RMIDtoRBUMP");
                AutoTrajectory RBUMPtoRTRENCH = routine.trajectory("RBUMPtoRTRENCH");
                AutoTrajectory RTRENCHtoHUBSWEEP = routine.trajectory("RTRENCHtoHUBSWEEP");

                routine.active().onTrue(Commands.sequence(RTRENCHtoRMID.resetOdometry(),
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                RTRENCHtoRMID.cmd()));

                RTRENCHtoRMID.done().onTrue(RMIDtoRBUMP.cmd());

                RMIDtoRBUMP.done()
                                .onTrue(Commands.sequence(RBUMPtoRTRENCH.cmd(),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SOTM_HUB_AUTO)));

                RBUMPtoRTRENCH.done().onTrue(Commands.sequence(
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                RTRENCHtoHUBSWEEP.cmd()));

                RTRENCHtoHUBSWEEP.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE)));

                return routine;
        }

        public AutoRoutine leftCheeseburger() {
                AutoRoutine routine = autoFactory.newRoutine("leftCheeseburger");

                AutoTrajectory LTRENCHtoLMID = routine.trajectory("LTRENCHtoLMID");
                AutoTrajectory LMIDtoLTRENCH = routine.trajectory("LMIDtoLTRENCH");
                AutoTrajectory LTRENCHtoDEPOT = routine.trajectory("LTRENCHtoDEPOT");
                AutoTrajectory DEPOT_INTAKE = routine.trajectory("DEPOT_INTAKE");

                routine.active().onTrue(Commands.sequence(LTRENCHtoLMID.resetOdometry(),
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                LTRENCHtoLMID.cmd()));

                LTRENCHtoLMID.done().onTrue(LMIDtoLTRENCH.cmd());

                LMIDtoLTRENCH.done().onTrue(LTRENCHtoDEPOT.cmd()
                                .alongWith(superstructureCommands
                                                .setWantedSuperStateCommand(WantedSuperState.SOTM_HUB_AUTO)));

                LTRENCHtoDEPOT.done().onTrue(
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO)
                                                .andThen(DEPOT_INTAKE.cmd()));

                DEPOT_INTAKE.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE)));

                return routine;
        }

        public AutoRoutine rightCheeseburger() {
                AutoRoutine routine = autoFactory.newRoutine("rightCheeseburger");

                AutoTrajectory RTRENCHtoRMID = routine.trajectory("RTRENCHtoRMID");
                AutoTrajectory RMIDtoRTRENCH = routine.trajectory("RMIDtoRTRENCH");
                AutoTrajectory RTRENCHtoOUTPOST = routine.trajectory("RTRENCHtoOUTPOST");
                AutoTrajectory OUTPOSTtoRTRENCH = routine.trajectory("OUTPOSTtoRTRENCH");

                routine.active().onTrue(Commands.sequence(RTRENCHtoRMID.resetOdometry(),
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                RTRENCHtoRMID.cmd()));

                RTRENCHtoRMID.done().onTrue(RMIDtoRTRENCH.cmd());

                RMIDtoRTRENCH.done()
                                .onTrue(Commands.sequence(
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                new WaitCommand(1),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(
                                                                                WantedSuperState.SHOOT_HUB_AND_AGITATE),
                                                new WaitCommand(2.5),
                                                superstructureCommands
                                                                .setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                                RTRENCHtoOUTPOST.cmd()));

                RTRENCHtoOUTPOST.done()
                                .onTrue(OUTPOSTtoRTRENCH.cmd());

                OUTPOSTtoRTRENCH.done()
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

                AutoTrajectory LBUMPtoDEPOT = routine.trajectory("LBUMPtoDEPOT");

                routine.active().onTrue(Commands.sequence(LBUMPtoDEPOT.resetOdometry(),
                                superstructureCommands.setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO),
                                LBUMPtoDEPOT.cmd()));

                LBUMPtoDEPOT.done()
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
