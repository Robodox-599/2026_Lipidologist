package frc.robot.subsystems.leds;

import com.ctre.phoenix6.signals.RGBWColor;

import dev.doglog.DogLog;

public class LEDs {
    private final LEDsIO io;
    private LEDsWantedState wantedState = LEDsWantedState.STOP;
    private LEDsCurrentState currentState = LEDsCurrentState.STOPPING;

    public LEDs(LEDsIO io) {
        this.io = io;
    }

    public enum LEDsWantedState {
        PREPARE_HUB_SHOT,
        SHOOT_HUB,
        PREPARE_ALLIANCE_ZONE_SHOT,
        SHOOT_ALLIANCE_ZONE,
        PREPARE_CLIMB,
        CLIMB,
        IDLE,
        STOP
    }

    public enum LEDsCurrentState {
        PREPARING_HUB_SHOT,
        SHOOTING_HUB,
        PREPARING_ALLIANCE_ZONE_SHOT,
        SHOOTING_ALLIANCE_ZONE,
        PREPARING_CLIMB,
        CLIMBING,
        IDLING,
        STOPPING
    }

    public void updateInputs() {
        io.updateInputs();

        handleStateTransitions();
        applyStates();

        DogLog.log("LEDs/wantedState", wantedState);
        DogLog.log("LEDs/currentState", currentState);
    }

    public void handleStateTransitions() {
        switch (wantedState) {
            case PREPARE_HUB_SHOT:
                currentState = LEDsCurrentState.PREPARING_HUB_SHOT;
                break;
            case SHOOT_HUB:
                currentState = LEDsCurrentState.SHOOTING_HUB;
                break;
            case PREPARE_ALLIANCE_ZONE_SHOT:
                currentState = LEDsCurrentState.PREPARING_ALLIANCE_ZONE_SHOT;
                break;
            case SHOOT_ALLIANCE_ZONE:
                currentState = LEDsCurrentState.SHOOTING_ALLIANCE_ZONE;
                break;
            case PREPARE_CLIMB:
                currentState = LEDsCurrentState.PREPARING_CLIMB;
                break;
            case CLIMB:
                currentState = LEDsCurrentState.CLIMBING;
                break;
            case IDLE:
                currentState = LEDsCurrentState.IDLING;
                break;
            case STOP:
                currentState = LEDsCurrentState.STOPPING;
                break;
            default:
                currentState = LEDsCurrentState.STOPPING;
                break;
        }
    }

    public void applyStates() {
        switch (currentState) {
            case PREPARING_HUB_SHOT:
                setLEDsColor(LEDsConstants.purple);
                break;
            case SHOOTING_HUB:
                setLEDsColor(LEDsConstants.green);
                break;
            case PREPARING_ALLIANCE_ZONE_SHOT:
                setLEDsColor(LEDsConstants.pink);
                break;
            case SHOOTING_ALLIANCE_ZONE:
                setLEDsColor(LEDsConstants.yellow);
                break;
            case PREPARING_CLIMB:
                setLEDsColor(LEDsConstants.magenta);
                break;
            case CLIMBING:
                setLEDsColor(LEDsConstants.lilac);
                break;
            case IDLING:
                setLEDsColor(LEDsConstants.lime);
                break;
            case STOPPING:
                setLEDsColor(LEDsConstants.red);
                break;
            default:
                setLEDsColor(LEDsConstants.red);
                break;
        }
    }

    public void setLEDsColor(RGBWColor color) {
        io.setLEDsColor(color);
    }

    public void setWantedState(LEDsWantedState wantedState) {
        this.wantedState = wantedState;
    }
}
