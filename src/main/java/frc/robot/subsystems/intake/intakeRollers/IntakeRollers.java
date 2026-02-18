package frc.robot.subsystems.intake.intakeRollers; // make sure the folder name is all lowercase

import dev.doglog.DogLog;

public class IntakeRollers {
    private final IntakeRollersIO io;
    private IntakeRollersWantedState wantedState = IntakeRollersWantedState.STOP;
    private IntakeRollersCurrentState currentState = IntakeRollersCurrentState.STOPPED;
    
    public IntakeRollers(IntakeRollersIO io){
        this.io = io;
    }

    public enum IntakeRollersWantedState{
        STOP,
        INTAKE_FUEL,
        REVERSE_FUEL
    }

    public enum IntakeRollersCurrentState{
        STOPPED,
        INTAKING_FUEL,
        REVERSING_FUEL
    }

    public void updateInputs(){
        io.updateInputs();
        handleStateTransitions();
        applyStates();
        DogLog.log("Intake/Rollers/WantedState", this.wantedState);
        DogLog.log("Intake/Rollers/CurrentState", this.currentState);
    }

    private void handleStateTransitions(){
        switch(wantedState){
            case STOP:
                currentState = IntakeRollersCurrentState.STOPPED;
                break;
            case INTAKE_FUEL:
                currentState = IntakeRollersCurrentState.INTAKING_FUEL;
                break;
            case REVERSE_FUEL:
                currentState = IntakeRollersCurrentState.REVERSING_FUEL;
                break;
            default:
                currentState = IntakeRollersCurrentState.STOPPED;
                break;
        }
    } 

    private void applyStates(){
        switch(currentState){
            case STOPPED:
                stop();
                break;
            case INTAKING_FUEL:
                setVoltage(10);
                break;
            case REVERSING_FUEL:
                setVoltage(-5);
                break;
            default:
                stop();
                break;
        }
    }

    public void stop(){
        io.stop();
    }

    public void setWantedState(IntakeRollers.IntakeRollersWantedState wantedState){
        this.wantedState = wantedState;
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
    }
}


