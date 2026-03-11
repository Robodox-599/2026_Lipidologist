package frc.robot.autos;

import java.util.List;
import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.util.HubShiftUtil;

public class AutoFactory {

        private static final Alert missingTrajectoryAlert = new Alert("One or more trajectories are missing!",
                        AlertType.kWarning);

        private static Command followTrajectory(String name, boolean start, CommandSwerveDrivetrain drivetrain,
                        Superstructure superstructure) {
                Optional<Trajectory<SwerveSample>> optionalTrajectory = Choreo.loadTrajectory(name);
                if (optionalTrajectory.isPresent()) {
                        Trajectory<SwerveSample> trajectory = optionalTrajectory.get();
                        return Commands.runOnce(() -> drivetrain.setDesiredChoreoTrajectory(trajectory, start))
                                        .andThen(new WaitUntilCommand(drivetrain::isAtEndOfChoreoTrajectory));
                } else {
                        missingTrajectoryAlert.set(true);
                        return Commands.none();
                }
        }

        public static Command followTrajectoryWhileIdle(String name, boolean start, CommandSwerveDrivetrain drivetrain,
                        Superstructure superstructure) {
                return Commands
                                .parallel(superstructure.setWantedSuperStateCommand(WantedSuperState.IDLE),
                                                followTrajectory(name, start, drivetrain, superstructure));
        }

        public static Command followTrajectoryWhileScoring(String name, boolean start,
                        CommandSwerveDrivetrain drivetrain,
                        Superstructure superstructure) {
                return Commands
                                .parallel(superstructure.setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                followTrajectory(name, start, drivetrain, superstructure));
        }

        public static Command followTrajectoryWhileScoringThenScore(String name, boolean start, double maxTime,
                        CommandSwerveDrivetrain drivetrain,
                        Superstructure superstructure) {
                return Commands
                                .parallel(superstructure.setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB),
                                                followTrajectory(name, start, drivetrain, superstructure))
                                .andThen(superstructure.setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB))
                                .andThen(new WaitCommand(maxTime));
        }

        public static Command followTrajectoryWhileIdleThenScore(String name, boolean start, double maxTime,
                        CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
                return followTrajectory(name, start, drivetrain, superstructure)
                                .alongWith(superstructure.setWantedSuperStateCommand(WantedSuperState.IDLE))
                                .andThen(superstructure.setWantedSuperStateCommand(WantedSuperState.SHOOT_HUB))
                                .andThen(new WaitCommand(maxTime));
        }

        public static double getTotalTrajectoryTime(String... names) {
                double time = 0;
                for (String name : names) {
                        Optional<Trajectory<SwerveSample>> optionalTrajectory = Choreo.loadTrajectory(name);
                        if (optionalTrajectory.isPresent()) {
                                time += optionalTrajectory.get().getTotalTime();
                        }
                }
                return time;
        }
}
