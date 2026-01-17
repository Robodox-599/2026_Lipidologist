package frc.robot.subsystems.intake.intakeRollers;

import dev.doglog.DogLog;

public class IntakeRollers {
    private IntakeRollersIO io;
    private WantedState wantedState = WantedState.STOPPED;
    private CurrentState currentState = CurrentState.STOPPED;
    
    public IntakeRollers(IntakeRollersIO io){
        this.io = io;
    }

    enum WantedState{
        STOPPED,
        INTAKING_FUEL,
        REVERSE_FUEL
    }

    enum CurrentState{
        STOPPED,
        INTAKING_FUEL,
        REVERSE_FUEL
    }

    public void updateInputs(){
        io.updateInputs();
        handleStateTransitions();
        applyStates();

        DogLog.log("WantedState", wantedState);
        DogLog.log("CurrentState", currentState);
    }

    public void handleStateTransitions(){
        switch(wantedState){
            case STOPPED:
                currentState = CurrentState.STOPPED;
                break;
            case INTAKING_FUEL:
                currentState = CurrentState.INTAKING_FUEL;
                break;
            case REVERSE_FUEL:
                currentState = CurrentState.REVERSE_FUEL;
                break;
            default:
                currentState = CurrentState.STOPPED;
                break;
        }
    } 

    public void applyStates(){
        switch(currentState){
            case STOPPED:
                stop();
                break;
            case INTAKING_FUEL:
                setVelocity(0);
                break;
            case REVERSE_FUEL:
                setVelocity(0);
                break;
        }
    }

    public void stop(){
        io.stop();
    }

    public void setVelocity(double velocity){
        io.setVelocity(velocity);
    }

    public void setWantedState(WantedState intakeRollersWantedState){
        this.wantedState = intakeRollersWantedState;
    }


}


