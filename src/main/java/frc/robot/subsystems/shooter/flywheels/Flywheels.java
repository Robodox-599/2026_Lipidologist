package frc.robot.subsystems.shooter.flywheels;

import dev.doglog.DogLog;

public class Flywheels {
    private final FlywheelsIO[] io;
    private FlywheelWantedState wantedState = FlywheelWantedState.STOPPED;
    private FlywheelCurrentState currentState = FlywheelCurrentState.STOPPING;

    private double targetRPS = 0;

    public Flywheels(FlywheelsIO... io) {
        this.io = io;
    }

    public enum FlywheelWantedState {
        SET_RPS,
        IDLE,
        STOPPED
    }

    public enum FlywheelCurrentState {
        SETTING_RPS,
        IDLING,
        STOPPING
    }

    public void updateInputs() {
        for (int i = 0; i < this.io.length; i++) {
            this.io[i].updateInputs();
        }
        handleStateTransitions();
        applyStates();

        DogLog.log("Flywheels/WantedState", wantedState);
        DogLog.log("Flywheels/CurrentState", currentState);
    }

    public void handleStateTransitions() {
        switch (wantedState) {
            case SET_RPS:
                currentState = FlywheelCurrentState.SETTING_RPS;
                break;
            case IDLE:
                currentState = FlywheelCurrentState.IDLING;
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
            case SETTING_RPS:
                setRPS(this.targetRPS);
                break;
            case IDLING:
                setRPS(FlywheelsConstants.idleRPS);
                break;
            case STOPPING:
                stop();
                break;
            default:
                stop();
                break;
        }
    }

    public void setRPS(double RPS) {
        for (int i = 0; i < this.io.length; i++) {
            this.io[i].setRPS(RPS);
        }
    }

    public void stop() {
        for (int i = 0; i < this.io.length; i++) {
            this.io[i].stop();
        }
    }

    public void setVoltage(double voltage) {
        for (int i = 0; i < this.io.length; i++) {
            this.io[i].setVoltage(voltage);
        }
    }

    public boolean atSetpoint() {
        boolean allFlywheelsAtSetpoint = true;
        for (int i = 0; i < this.io.length; i++) {
            allFlywheelsAtSetpoint = allFlywheelsAtSetpoint && this.io[i].isFlywheelAtSetpoint;
        }
        return allFlywheelsAtSetpoint;
    }

    public void setWantedState(Flywheels.FlywheelWantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setWantedState(Flywheels.FlywheelWantedState wantedState, double targetRPS) {
        this.wantedState = wantedState;
        this.targetRPS = targetRPS;
    }
}
