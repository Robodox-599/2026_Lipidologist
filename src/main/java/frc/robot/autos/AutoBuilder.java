package frc.robot.autos;

import choreo.auto.AutoTrajectory;
import dev.doglog.DogLog;
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
        double totalTrajTime = AutoFactory.getTotalTrajectoryTime("LTRENCHtoLMID", "LMIDtoLTRENCH", "LTRENCHtoHUBSWEEP");
        DogLog.log("Auto/totalTrajectoryTime", totalTrajTime);
        double finalShootTime = 5.0;
        return Commands.sequence(
                AutoFactory.followTrajectoryWhileIdle("LTRENCHtoLMID", true, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileIdleThenScore("LMIDtoLTRENCH", false, 20.0 - totalTrajTime - finalShootTime, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileIdleThenScore("LTRENCHtoHUBSWEEP", false, 10.0, this.drivetrain, this.superstructure));
    }
    
    public Command rightDoubleDouble() {
        double totalTrajTime = AutoFactory.getTotalTrajectoryTime("RTRENCHtoRMID", "RMIDtoRTRENCH", "RTRENCHtoHUBSWEEP");
        DogLog.log("Auto/totalTrajectoryTime", totalTrajTime);
        return Commands.sequence(
                AutoFactory.followTrajectoryWhileIdle("RTRENCHtoRMID", true, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileIdleThenScore("RMIDtoRTRENCH", false, 3, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileIdleThenScore("RTRENCHtoHUBSWEEP", false, 10.0, this.drivetrain, this.superstructure));
    }

    public Command leftCheeseBurger() {
        double totalTrajTime = AutoFactory.getTotalTrajectoryTime("RTRENCHtoRMID", "RMIDtoRTRENCH", "RTRENCHtoHUBSWEEP");
        double finalShootTime = 5.0;
        DogLog.log("Auto/totalTrajectoryTime", totalTrajTime);
        return Commands.sequence(
                AutoFactory.followTrajectoryWhileIdle("LTRENCHtoLMID", true, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileIdle("LMIDtoLTRENCH", false, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileScoringThenScore("LTRENCHtoDEPOT", false, 20.0 - totalTrajTime - finalShootTime,this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileIdleThenScore("DEPOT_INTAKE", false, finalShootTime, this.drivetrain, this.superstructure));
    }

    public Command rightCheeseBurger() {
        double totalTrajTime = AutoFactory.getTotalTrajectoryTime("RTRENCHtoRMID", "RMIDtoRTRENCH", "RTRENCHtoOUTPOST");
        DogLog.log("Auto/totalTrajectoryTime", totalTrajTime);
        return Commands.sequence(
                AutoFactory.followTrajectoryWhileIdle("RTRENCHtoRMID", true, this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileIdle("RMIDtoRTRENCH", false,  this.drivetrain, this.superstructure),
                AutoFactory.followTrajectoryWhileScoringThenScore("RTRENCHtoOUTPOST", false, 20.0 - totalTrajTime, this.drivetrain, this.superstructure));
    }
}
