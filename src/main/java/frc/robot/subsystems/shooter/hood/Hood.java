package frc.robot.subsystems.shooter.hood;

import dev.doglog.DogLog;
import frc.robot.util.Tracer;

public class Hood {
    private final HoodIO io;
    private HoodWantedState wantedState = HoodWantedState.STOPPED;
    private HoodCurrentState currentState = HoodCurrentState.STOPPING;

    public Hood(HoodIO io) {
        this.io = io;
    }

    public enum HoodWantedState {
        SET_POSITION,
        STOPPED
    }

    public enum HoodCurrentState {
        SETTING_POSITION,
        STOPPING
    }

    public void updateInputs() {
        Tracer.traceFunc("HoodUpdateInputs", io::updateInputs);
        handleStateTransitions();
        applyStates();

        DogLog.log("Hood/WantedState", wantedState);
        DogLog.log("Hood/CurrentState", currentState);
    }

    public void handleStateTransitions() {
        switch (wantedState) {
            case SET_POSITION:
                currentState = HoodCurrentState.SETTING_POSITION;
                break;
            case STOPPED:
                currentState = HoodCurrentState.STOPPING;
                break;
            default:
                currentState = HoodCurrentState.STOPPING;
                break;
        }
    }

    private void applyStates() {
        switch (currentState) {
            case SETTING_POSITION:
                setPosition(io.targetPositionRots);
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

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public boolean atSetpoint() {
        return io.isHoodInPosition;
    }

    public void setWantedState(Hood.HoodWantedState WantedState) {
        this.wantedState = WantedState;
    }

    public void setWantedState(Hood.HoodWantedState WantedState, double position) {
        this.wantedState = WantedState;
        io.targetPositionRots = position;
    }
}
