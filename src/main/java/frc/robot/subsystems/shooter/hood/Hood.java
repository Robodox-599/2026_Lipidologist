package frc.robot.subsystems.shooter.hood;

import dev.doglog.DogLog;

public class Hood extends HoodIO{
    private final HoodIO io;
    private WantedState wantedState = WantedState.STOPPED;
    private CurrentState currentState = CurrentState.STOPPING;

    public Hood(HoodIO io) {
        this.io = io;
    }

    public enum WantedState {
        SET_POSITION,
        STOPPED
    }

    public enum CurrentState {
        SETTING_POSITION,
        STOPPING 
    }

    public void updateInputs() {
        io.updateInputs();
        handleStateTransitions();
        applyStates();

        DogLog.log("Hood/WantedState", wantedState);
        DogLog.log("Hood/CurrentState", currentState);
    }

    public void handleStateTransitions() {
        switch(wantedState) {
            case SET_POSITION:
            currentState = CurrentState.SETTING_POSITION;
            break;
            case STOPPED:
            currentState = CurrentState.STOPPING;
            break;
            default:
            currentState = CurrentState.STOPPING;
            break;
        }
    }

    private void applyStates() {
        switch (currentState) {
            case SETTING_POSITION:
            setPosition(io.targetPosition);
            break;
            case STOPPING:
            stop();
            break;
            default:
            stop();
            break;
        }
    }

    public void setPosition(double position) {
        io.setPosition(position);
    }
    
    public void stop() {
        io.stop();
    }
    
    public boolean isHoodAtSetpoint() {
        return io.isHoodInPosition;
    }
    
    public void setWantedState(Hood.WantedState WantedState){
      this.wantedState = WantedState;
    }

    public void setWantedState(Hood.WantedState WantedState, double position){
      this.wantedState = WantedState;
      io.targetPosition = position;
    }
}
