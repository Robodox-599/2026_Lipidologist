package frc.robot.subsystems.shooter.hood;

import dev.doglog.DogLog;

public class Hood extends HoodIO{
    private final HoodIO io;
    private WantedState wantedState = WantedState.STOPPED;
    private CurrentState currentState = CurrentState.STOPPED;

    public Hood(HoodIO io) {
        this.io = io;
    }

    public enum WantedState {
        MOVE_TO_POSITION,
        STOPPED
    }

    public enum CurrentState {
        MOVING_TO_POSITION,
        STOPPED // rename this to STOPPING
    }

    public void updateInputs() {
        io.updateInputs();
        handleStateTransitions();
        applyStates();
        // get rid of these random spaces
        DogLog.log("Hood/Wanted State", wantedState);
        DogLog.log("Hood/Current State", currentState);
    }

    public void handleStateTransitions() {
        switch(wantedState) {
            case MOVE_TO_POSITION:
            currentState = CurrentState.MOVING_TO_POSITION;
            break;
            case STOPPED:
            currentState = CurrentState.STOPPED;
            break;
            default:
            currentState = CurrentState.STOPPED;
            break;
        }
    }

    private void applyStates() {
        switch (currentState) {
            case MOVING_TO_POSITION:
            setPosition(io.targetPosition);
            break;
            case STOPPED:
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
    
    // why are there two setWantedState functions? Get rid of whichever one you don't use
    public void setWantedState(Hood.WantedState WantedState){
      this.wantedState = WantedState;
    }

    public void setWantedState(Hood.WantedState WantedState, double position){
      this.wantedState = WantedState;
      io.targetPosition = position;
    }
}
