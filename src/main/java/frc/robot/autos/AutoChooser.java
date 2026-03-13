// package frc.robot.autos;

// import java.util.ArrayList;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;

// public class AutoChooser {
//     public enum AutoMode {
//         DOUBLE_DOUBLE,
//         CHEESEBURGER_WITH_ONIONS,
//         NOTHING,
//     }

//     public enum StartLocation {
//         LEFT,
//         RIGHT,
//     }

//     private final SendableChooser<AutoMode> routineChooser = new SendableChooser<>();
//     private final SendableChooser<StartLocation> startingLocationChooser = new SendableChooser<>();
//     private final AutoBuilder autoBuilder;
//     private AutoMode autoMode = AutoMode.NOTHING;
//     private StartLocation startLocation = StartLocation.LEFT;
//     private Command autoCommand = Commands.none();

//     public AutoChooser(AutoBuilder autoBuilder) {
//         this.autoBuilder = autoBuilder;

//         routineChooser.setDefaultOption("NOTHING", AutoMode.NOTHING);
//         routineChooser.addOption("DOUBLE_DOUBLE", AutoMode.DOUBLE_DOUBLE);
//         routineChooser.addOption("CHEESEBURGER_WITH_ONIONS", AutoMode.CHEESEBURGER_WITH_ONIONS);

//         startingLocationChooser.setDefaultOption("LEFT", StartLocation.LEFT);
//         startingLocationChooser.addOption("RIGHT", StartLocation.RIGHT);
//     }

//     public SendableChooser<AutoMode> getRoutineChooser() {
//         return routineChooser;
//     }

//     public SendableChooser<StartLocation> getStartingLocationChooser() {
//         return startingLocationChooser;
//     }

//     public Command getSelectedAutoCommand() {
//         return autoCommand;
//     }

//     public void updateAutoChooser() {
//         // Update the list of questions
//         AutoMode chosenAutoMode = routineChooser.getSelected();
//         StartLocation chosenStartLocation = startingLocationChooser.getSelected();

//         if (chosenAutoMode == null) {
//             chosenAutoMode = AutoMode.NOTHING;
//         }
//         if (chosenStartLocation == null) {
//             chosenStartLocation = StartLocation.LEFT;
//         }

//         if (chosenAutoMode != this.autoMode || chosenStartLocation != this.startLocation || autoCommand == null) {
//             this.autoMode = chosenAutoMode;
//             this.startLocation = chosenStartLocation;
//             autoCommand = getSelectedCommand(this.autoMode, this.startLocation);
//         }
//     }

//     private Command getSelectedCommand(AutoMode chosenAutoMode, StartLocation chosenStartLocation) {
//         switch (chosenAutoMode) {
//             case DOUBLE_DOUBLE:
//                 return (chosenStartLocation == StartLocation.LEFT)
//                         ? autoBuilder.leftDoubleDouble()
//                         : autoBuilder.rightDoubleDouble();
//             case CHEESEBURGER_WITH_ONIONS:
//                 return (chosenStartLocation == StartLocation.LEFT)
//                         ? autoBuilder.leftCheeseBurger()
//                         : autoBuilder.rightCheeseBurger();
//             case NOTHING:
//                 return Commands.none();
//             default:
//                 return Commands.none();
//         }
//     }
// }
