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

    public Command leftFarMidDepotAuto() {
        return Commands.sequence(
                AutoFactory.followTrajectoryWhileIdle("LTRENCHtoLMID", true, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileIdle("LMIDtoLTRENCH", false, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileScoring("LTRENCHtoDEPOT", false, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileIdle("DEPOT_INTAKE", false, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileIdle("DEPOTtoLTOWER", false, this.drivetrain, this.superstructure));
    }
}
