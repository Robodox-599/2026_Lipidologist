package frc.robot.subsystems.shooter.flywheels;

import dev.doglog.DogLog;

public class Flywheels extends FlywheelsIO{
    private final FlywheelsIO io;
    private FlywheelWantedState flywheelWantedState = FlywheelWantedState.STOPPED;
    private FlywheelCurrentState flywheelCurrentState = FlywheelCurrentState.STOPPED;

    public Flywheels(FlywheelsIO io) {
        this.io = io;
    }

    public enum FlywheelWantedState {
        SET_VELOCITY,
        STOPPED
    }

    public enum FlywheelCurrentState {
        SETTING_VELOCITY,
        STOPPED
    }
    
    public void updateInputs() {
        io.updateInputs();
        handleStateTransitions();
        applyStates();
        DogLog.log("Flywheel/Wanted State", flywheelWantedState);
        DogLog.log("Flywheel/Current State", flywheelCurrentState);
    }

    public void handleStateTransitions() {
        switch(flywheelWantedState) {
            case SET_VELOCITY:
            flywheelCurrentState = FlywheelCurrentState.SETTING_VELOCITY;
            break;
            case STOPPED:
            flywheelCurrentState = FlywheelCurrentState.STOPPED;
            break;
            default:
            flywheelCurrentState = FlywheelCurrentState.STOPPED;
            break;
        }
    }

    private void applyStates() {
        switch (flywheelCurrentState) {
            case SETTING_VELOCITY:
            setRPM(0);
            break;
            case STOPPED:
            stop();
            break;
            default:
            stop();
            break;
        }
    }

    public void setRPM(double rpm) {
        io.setRPM(rpm);
    }
    
    public void stop() {
        io.stop();
    }
    
    
    public boolean flywheelsAtSetpoint() {
        return io.isFlywheelAtSetpoint;
    }
    
    public void setFlywheelWantedState(FlywheelWantedState flywheelWantedState){
      this.flywheelWantedState = flywheelWantedState;
    }

    public void setFlywheelWantedState(FlywheelWantedState flywheelWantedState, double rpm){
      this.flywheelWantedState = flywheelWantedState;
    //   io.setRPM(rpm);
    // io.targetRPM = rpm;
    }
}
