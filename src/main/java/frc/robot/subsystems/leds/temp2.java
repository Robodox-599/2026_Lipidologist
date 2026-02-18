package frc.robot.subsystems.leds;

import dev.doglog.DogLog;

public class temp2 {
    private final poo3 io;
    private LedWantedState ledWantedState = LedWantedState.STOP;
    private LedCurrentState ledCurrentState = LedCurrentState.STOPPED;

    public enum LedWantedState{
        PREPARE_HUB_SHOT,
        SHOOT_HUB,
        PREPARE_ALLIANCE_ZONE_SHOT,
        SHOOT_ALLIANCE_ZONE,
        IDLE,
        STOP,
    }

    public enum LedCurrentState{
        PREPARING_HUB_SHOT,
        SHOOTING_HUB,
        PREPARING_ALLIANCE_ZONE_SHOT,
        SHOOTING_ALLIANCE_ZONE,
        IDLING,
        STOPPED;
    }

    public temp2(poo3 io){
        this.io = io;
    }

    public void updateInputs(){
        handleStateTransitions();
        applyStates();
        DogLog.log("LED/WantedState", this.ledWantedState);
        DogLog.log("LED/CurrentState", this.ledCurrentState);
    }

    public void handleStateTransitions(){
        switch(ledWantedState){
            case PREPARE_HUB_SHOT:
                ledCurrentState = LedCurrentState.PREPARING_HUB_SHOT;
                break;
            case SHOOT_HUB:
                ledCurrentState = LedCurrentState.SHOOTING_HUB;
                break;
            case PREPARE_ALLIANCE_ZONE_SHOT:
                ledCurrentState = LedCurrentState.PREPARING_ALLIANCE_ZONE_SHOT;
                break;
            case SHOOT_ALLIANCE_ZONE:
                ledCurrentState = LedCurrentState.SHOOTING_ALLIANCE_ZONE;
                break;
            case IDLE:
                ledCurrentState = LedCurrentState.IDLING;
                break;
            case STOP:
                ledCurrentState = LedCurrentState.STOPPED;
                break;
            default:
                ledCurrentState = LedCurrentState.STOPPED;
                break;
        }
    }

    public void applyStates(){
        switch(ledCurrentState){
            case PREPARING_HUB_SHOT:
                setLedColor(0, 0, 0);
                break;
            case SHOOTING_HUB:
                setLedColor(0, 0, 0);
                break;
            case PREPARING_ALLIANCE_ZONE_SHOT:
                setLedColor(0, 0, 0);
                break;
            case SHOOTING_ALLIANCE_ZONE:
                setLedColor(0, 0, 0);
                break;
            case IDLING:
                setLedColor(0, 0, 0);
                break;
            case STOPPED:
                setLedColor(0, 0, 0); //should be off
                break;
            default:
                setLedColor(0, 0, 0);
                break;

        }
    }

    public void setLedColor(int red, int green, int blue){
        io.setLedColor(red, green, blue);
    }

    public void setWantedState(temp2.LedWantedState ledWantedState){
        this.ledWantedState = ledWantedState;
    }
}
