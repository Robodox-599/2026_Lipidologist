package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.util.HubShiftUtil;

public class AutoRoutines {
 private AutoFactory autoFactory;
  private Superstructure superstructure;
  private CommandSwerveDrivetrain drivetrain;

  public AutoRoutines(
      AutoFactory autoFactory,
      Superstructure superstructure,
      CommandSwerveDrivetrain drivetrain) {
    this.autoFactory = autoFactory;
    this.superstructure = superstructure;
    this.drivetrain = drivetrain;
  }

  public AutoRoutine leftAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("leftAuto");

    AutoTrajectory LTROUGHtoLMID = routine.trajectory("LTROUGHtoLMID");
    AutoTrajectory LMIDtoLTROUGH = routine.trajectory("LMIDtoLTROUGH");
    AutoTrajectory LTROUGHtoDEPOT = routine.trajectory("LTROUGHtoDEPOT");
    AutoTrajectory DEPOT_INTAKE = routine.trajectory("DEPOT_INTAKE");
    AutoTrajectory DEPOTtoLTOWER = routine.trajectory("DEPOTtoLTOWER");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                LTROUGHtoLMID.resetOdometry(),
                superstructure.setWantedSuperStateCommand(
                    WantedSuperState.IDLE_AUTO),
                LTROUGHtoLMID.cmd()));

    LTROUGHtoLMID.done()
        .onTrue(LMIDtoLTROUGH.cmd());

    LMIDtoLTROUGH.done()
        .onTrue(LTROUGHtoDEPOT.cmd().alongWith(superstructure.setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB_AUTO)));

    LTROUGHtoDEPOT.done().onTrue(DEPOT_INTAKE.cmd().alongWith(superstructure.setWantedSuperStateCommand(WantedSuperState.IDLE_AUTO)));

    DEPOT_INTAKE.done().onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB));

    DEPOT_INTAKE.recentlyDone().and(new Trigger(() -> HubShiftUtil.getShiftInfo().remainingTime() < 5.0)).onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.CLIMB));

    return routine;
  }
  
  public AutoRoutine sweepAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("sweepAutoRoutine");

    AutoTrajectory RTROUGHtoRMIDSWEEP = routine.trajectory("RTROUGHtoRMIDSWEEP");
    AutoTrajectory RSWEEPtoRTROUGH = routine.trajectory("RSWEEPtoRTROUGH");
    AutoTrajectory RTROUGHtoRHUBSWEEP = routine.trajectory("RTROUGHtoRHUBSWEEP");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                RTROUGHtoRMIDSWEEP.resetOdometry(),
                superstructure.setWantedSuperStateCommand(
                    WantedSuperState.IDLE_AUTO),
                RTROUGHtoRMIDSWEEP.cmd()));

    RTROUGHtoRMIDSWEEP.done()
        .onTrue(RSWEEPtoRTROUGH.cmd());

    RSWEEPtoRTROUGH.done()
        .onTrue(RTROUGHtoRHUBSWEEP.cmd());

    return routine;
  }
}
