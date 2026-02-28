package frc.robot.autos;

import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoBuilder {
    private final CommandSwerveDrivetrain drivetrain;
    private final Superstructure superstructure;

    public AutoBuilder(CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
        this.drivetrain = drivetrain;
        this.superstructure = superstructure;
    }

    
}
