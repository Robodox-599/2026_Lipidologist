package frc.robot.subsystems.shooter.flywheels;

import dev.doglog.DogLog;

public class Flywheels extends FlywheelsIO{
    private final FlywheelsIO io;
    private WantedState wantedState = WantedState.STOPPED;
    private CurrentState currentState = CurrentState.STOPPED;

    public Flywheels(FlywheelsIO io) {
        this.io = io;
    }

    public enum WantedState {
        SET_VELOCITY,
        STOPPED
    }

    public enum CurrentState {
        SETTING_VELOCITY,
        STOPPED // rename this to STOPPING
    }
    
    public void updateInputs() {
        io.updateInputs();
        handleStateTransitions();
        applyStates();
        // make sure to get rid of these spaces; could cause errors
        DogLog.log("Flywheel/Wanted State", wantedState);
        DogLog.log("Flywheel/Current State", currentState);
    }

    public void handleStateTransitions() {
        switch(wantedState) {
            case SET_VELOCITY:
            currentState = CurrentState.SETTING_VELOCITY;
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
            case SETTING_VELOCITY:
            setRPM(io.targetRPM);
            break;
            case STOPPED:
            stop();
            break;
            default:
            stop();
            break;
        }
    }

    public void setRPM(double RPM) {
        io.setRPM(RPM);
    }
    
    public void stop() {
        io.stop();
    }
    
    
    public boolean flywheelsAtSetpoint() {
        return io.isFlywheelAtSetpoint;
    }
    
    // why are there two setWantedState functions? Get rid of whichever one you don't use
    public void setWantedState(Flywheels.WantedState WantedState){
      this.wantedState = WantedState;
    }

    public void setWantedState(Flywheels.WantedState WantedState, double RPM){
      this.wantedState = WantedState;
      io.targetRPM = RPM;
    }
}
