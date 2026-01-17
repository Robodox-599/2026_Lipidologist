package frc.robot.subsystems.shooter.hood;

import dev.doglog.DogLog;

public class Hood extends HoodIO{
    private final HoodIO io;
    private HoodWantedState hoodWantedState = HoodWantedState.STOPPED;
    private HoodCurrentState hoodCurrentState = HoodCurrentState.STOPPED;

    public Hood(HoodIO io) {
        this.io = io;
    }

    public enum HoodWantedState {
        MOVE_TO_POSITION,
        STOPPED
    }

    public enum HoodCurrentState {
        MOVING_TO_POSITION,
        STOPPED
    }

    public void updateInputs() {
        io.updateInputs();
        handleStateTransitions();
        applyStates();
        DogLog.log("Hood/Wanted State", hoodWantedState);
        DogLog.log("Hood/Current State", hoodCurrentState);
    }

    public void handleStateTransitions() {
        switch(hoodWantedState) {
            case MOVE_TO_POSITION:
            hoodCurrentState = HoodCurrentState.MOVING_TO_POSITION;
            break;
            case STOPPED:
            hoodCurrentState = HoodCurrentState.STOPPED;
            break;
            default:
            hoodCurrentState = HoodCurrentState.STOPPED;
            break;
        }
    }

    private void applyStates() {
        switch (hoodCurrentState) {
            case MOVING_TO_POSITION:
            setPosition(0);
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
}
