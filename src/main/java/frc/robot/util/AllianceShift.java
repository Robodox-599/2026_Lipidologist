package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class AllianceShift {
    private static Timer shiftTimer = new Timer();

    private static final double[] shiftStartTimes = { 0.0, 10.0, 35.0, 60.0, 85.0, 110.0 };
    private static final double[] shiftEndTimes = { 10.0, 35.0, 60.0, 85.0, 110.0, 140.0 };
    private static final boolean[] activeSchedule = { true, true, false, true, false, true };
    private static final boolean[] inactiveSchedule = { true, false, true, false, true, true };

    private static Alliance getFirstActiveAlliance() {
        String message = DriverStation.getGameSpecificMessage();
        if (message.length() > 0) {
            char character = message.charAt(0);
            if (character == 'R') {
                return Alliance.Blue;
            } else if (character == 'B') {
                return Alliance.Red;
            }
        }
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? Alliance.Red
                : Alliance.Blue;
    }

    /** Starts the timer at the begining of teleop. */
    public static void initialize() {
        shiftTimer.restart();
    }

    public static boolean isHubActive() {
        double currentTime = shiftTimer.get();
        Alliance startAlliance = getFirstActiveAlliance();
        boolean[] currentSchedule = startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
                ? activeSchedule
                : inactiveSchedule;
        if (DriverStation.isAutonomous()) {
            return true;
        } else if (DriverStation.isEnabled()) {
            int currentShiftIndex = -1;
            for (int i = 0; i < shiftStartTimes.length; i++) {
                if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
                    currentShiftIndex = i;
                    break;
                }
            }
            if (currentShiftIndex < 0) {
                // After last shift, so assume endgame
                return true;
            }
            return currentSchedule[currentShiftIndex];
        } else {
            return false;
        }
    }

    public static boolean isHubPredictedActive(double flightTime) {
        double currentTime = shiftTimer.get();
        Alliance startAlliance = getFirstActiveAlliance();
        boolean[] currentSchedule = startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
                ? activeSchedule
                : inactiveSchedule;
        if (DriverStation.isAutonomous()) {
            return true;
        } else if (DriverStation.isEnabled()) {
            int currentShiftIndex = -1;
            for (int i = 0; i < shiftStartTimes.length; i++) {
                if (currentTime + flightTime >= shiftStartTimes[i] && currentTime - flightTime < shiftEndTimes[i]) {
                    currentShiftIndex = i;
                    break;
                }
            }
            if (currentShiftIndex < 0) {
                // After last shift, so assume endgame
                return true;
            }
            return currentSchedule[currentShiftIndex];
        } else {
            return false;
        }
    }

    
}
