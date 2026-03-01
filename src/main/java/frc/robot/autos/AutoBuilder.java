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
                AutoFactory.followTrajectory("LTRENCHtoLMID", true, drivetrain, superstructure),
                AutoFactory.followTrajectory("LMIDtoLTRENCH", false, drivetrain, superstructure),
                AutoFactory.followTrajectoryWhileScoring("LTRENCHtoDEPOT", false, drivetrain, superstructure),
                AutoFactory.followTrajectory("LTRENCHtoDEPOT", false, drivetrain, superstructure));
    }
}
