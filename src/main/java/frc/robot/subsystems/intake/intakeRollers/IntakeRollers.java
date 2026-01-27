package frc.robot.subsystems.intake.intakeRollers; // make sure the folder name is all lowercase

import dev.doglog.DogLog;

public class IntakeRollers {
    private final IntakeRollersIO io;
    private WantedState wantedState = WantedState.STOP;
    private CurrentState currentState = CurrentState.STOPPED;
    
    public IntakeRollers(IntakeRollersIO io){
        this.io = io;
    }

    public enum WantedState{
        STOP,
        INTAKE_FUEL,
        REVERSE_FUEL
    }

    public enum CurrentState{
        STOPPED,
        INTAKING_FUEL,
        REVERSING_FUEL
    }

    public void updateInputs(){
        io.updateInputs();
        handleStateTransitions();
        applyStates();
        DogLog.log("Intake/Rollers/WantedState", wantedState);
        DogLog.log("Intake/Rollers/CurrentState", currentState);
    }

    private void handleStateTransitions(){
        switch(wantedState){
            case STOP:
                currentState = CurrentState.STOPPED;
                break;
            case INTAKE_FUEL:
                currentState = CurrentState.INTAKING_FUEL;
                break;
            case REVERSE_FUEL:
                currentState = CurrentState.REVERSING_FUEL;
                break;
            default:
                currentState = CurrentState.STOPPED;
                break;
        }
    } 

    private void applyStates(){
        switch(currentState){
            case STOPPED:
                stop();
                break;
            case INTAKING_FUEL:
                setVoltage(0);
                break;
            case REVERSING_FUEL:
                setVoltage(0);
                break;
            default:
                stop();
                break;
        }
    }

    public void stop(){
        io.stop();
    }

    public void setVelocity(double velocity){
        io.setVelocity(velocity);
    }

    public void setWantedState(IntakeRollers.WantedState wantedState){
        this.wantedState = wantedState;
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
    }
}


