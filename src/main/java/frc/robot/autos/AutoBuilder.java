package frc.robot.autos;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoBuilder {
    private final CommandSwerveDrivetrain drivetrain;
    private final Superstructure superstructure;

    public AutoBuilder(CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
        this.drivetrain = drivetrain;
        this.superstructure = superstructure;
    }

    public Command leftDoubleDouble() {
        return Commands.sequence(
                AutoFactory.followTrajectoryWhileIdle("LTRENCHtoLMID", true, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileIdle("LMIDtoLTRENCH", false, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileScoring("LTRENCHtoDEPOT", false, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileIdle("DEPOT_INTAKE", false, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileIdle("DEPOTtoLTOWER", false, this.drivetrain, this.superstructure));
    }
    
    public Command rightDoubleDouble() {
        return Commands.sequence(
                AutoFactory.followTrajectoryWhileIdle("RTRENCHtoRMID", true, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryThenScore("RMIDtoRTRENCH", false, 5, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryThenScore("RTRENCHtoRHUBSWEEP", false, 10, this.drivetrain, this.superstructure));
    }

    public Command leftCheeseBurgerWithOnions() {
        return Commands.sequence(
                AutoFactory.followTrajectoryWhileIdle("LTRENCHtoLMID", true, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileIdle("LMIDtoLTRENCH", false, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileScoring("LTRENCHtoDEPOT", false, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileIdle("DEPOT_INTAKE", false, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileIdle("DEPOTtoLTOWER", false, this.drivetrain, this.superstructure));
    }

    public Command rightFarMidOutpostAuto() {
        return Commands.sequence(
                AutoFactory.followTrajectoryWhileIdle("RTRENCHtoRMID", true, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryThenScore("RMIDtoRTRENCH", false, 5, this.drivetrain, this.superstructure));
                // AutoFactory.followTrajectoryWhileScoring("RTRENCHtoOUTPOST", false, this.drivetrain, this.superstructure));
                // AutoFactory.followTrajectoryWhileIdle("DEPOT_INTAKE", false, this.drivetrain, this.superstructure),
                // AutoFactory.followTrajectoryWhileIdle("DEPOTtoLTOWER", false, this.drivetrain, this.superstructure));
    }
}
