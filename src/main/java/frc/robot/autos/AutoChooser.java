package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoChooser {
    public enum AutoMode {
        DOUBLE_DOUBLE,
        CHEESEBURGER_WITH_ONIONS,
        NOTHING,
    }

    public enum StartLocation {
        LEFT,
        RIGHT,
    }

    private final SendableChooser<AutoMode> routineChooser = new SendableChooser<>();
    private final SendableChooser<StartLocation> startingLocationChooser = new SendableChooser<>();
    private final AutoBuilder autoBuilder;

    public AutoChooser(AutoBuilder autoBuilder) {
        this.autoBuilder = autoBuilder;

        routineChooser.setDefaultOption("NOTHING", AutoMode.NOTHING);
        routineChooser.addOption("DOUBLE_DOUBLE", AutoMode.DOUBLE_DOUBLE);
        routineChooser.addOption("CHEESEBURGER_WITH_ONIONS", AutoMode.CHEESEBURGER_WITH_ONIONS);

        startingLocationChooser.setDefaultOption("LEFT", StartLocation.LEFT);
        startingLocationChooser.addOption("RIGHT", StartLocation.RIGHT);
    }

    public SendableChooser<AutoMode> getRoutineChooser() {
        return routineChooser;
    }

    public SendableChooser<StartLocation> getStartingLocationChooser() {
        return startingLocationChooser;
    }

    public Command getSelectedCommand() {
        AutoMode selected = routineChooser.getSelected();

        if (selected == null) {
            selected = AutoMode.NOTHING;
        }
        switch (selected) {
            case DOUBLE_DOUBLE:
                return (startingLocationChooser.getSelected() == StartLocation.LEFT) ? autoBuilder.leftDoubleDouble() : autoBuilder.rightDoubleDouble();
            case CHEESEBURGER_WITH_ONIONS:
                return (startingLocationChooser.getSelected() == StartLocation.LEFT) ? autoBuilder.leftCheeseBurgerWithOnions() : autoBuilder.leftCheeseBurgerWithOnions();
            case NOTHING: 
                return Commands.none();
            default:
                return Commands.none();
        }
    }
}
