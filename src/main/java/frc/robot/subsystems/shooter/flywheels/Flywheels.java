package frc.robot.subsystems.shooter.flywheels;

import dev.doglog.DogLog;

public class Flywheels extends FlywheelsIO{
    private final FlywheelsIO io;
    private FlywheelWantedState wantedState = FlywheelWantedState.STOPPED;
    private FlywheelCurrentState currentState = FlywheelCurrentState.STOPPING;

    public Flywheels(FlywheelsIO io) {
        this.io = io;
    }

    public enum FlywheelWantedState {
        SET_RPM,
        SET_CONSTANT_RPM,
        STOPPED
    }

    public enum FlywheelCurrentState {
        SETTING_RPM,
        SETTING_CONSTANT_RPM,
        STOPPING
    }
    
    public void updateInputs() {
        io.updateInputs();
        handleStateTransitions();
        applyStates();

        DogLog.log("Flywheel/WantedState", wantedState);
        DogLog.log("Flywheel/CurrentState", currentState);
    }

    public void handleStateTransitions() {
        switch(wantedState) {
            case SET_RPM:
            currentState = FlywheelCurrentState.SETTING_RPM;
            break;
            case SET_CONSTANT_RPM:
            currentState = FlywheelCurrentState.SETTING_CONSTANT_RPM;
            break;
            case STOPPED:
            currentState = FlywheelCurrentState.STOPPING;
            break;
            default:
            currentState = FlywheelCurrentState.STOPPING;
            break;
        }
    }

    private void applyStates() {
        switch (currentState) {
            case SETTING_RPM:
            setRPM(io.targetRPM);
            break;
            case SETTING_CONSTANT_RPM:
            setRPM(FlywheelsConstants.constantRPMSetpoint);
            break;
            case STOPPING:
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
    
     public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }
    
    public boolean atSetpoint() {
        return io.isFlywheelAtSetpoint;
    }
    
    public void setWantedState(Flywheels.FlywheelWantedState WantedState){
      this.wantedState = WantedState;
    }

    public void setWantedState(Flywheels.FlywheelWantedState WantedState, double RPM){
      this.wantedState = WantedState;
      io.targetRPM = RPM;
    }
}
