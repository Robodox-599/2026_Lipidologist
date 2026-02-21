package frc.robot.subsystems.shooter.flywheels;

import dev.doglog.DogLog;

public class Flywheels {
    private final FlywheelsIO io;
    private FlywheelWantedState wantedState = FlywheelWantedState.STOPPED;
    private FlywheelCurrentState currentState = FlywheelCurrentState.STOPPING;

    private double targetRPS = 0;

    public Flywheels(FlywheelsIO io) {
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
        io.updateInputs();
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
                setRPS(targetRPS);
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
        io.setRPS(RPS);
    }

    public void stop() {
        io.stop();
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public boolean atSetpoint() {
        boolean allFlywheelsAtSetpoint = true;
        if(io.isFlywheelAtSetpointRight && io.isFlywheelAtSetpointCenter && io.isFlywheelAtSetpointLeft) {
            allFlywheelsAtSetpoint = true;
        } else {
            allFlywheelsAtSetpoint = false;
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
