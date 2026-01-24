// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeWrist;

import dev.doglog.DogLog;

/** Add your docs here. */
public class IntakeWrist {
    private IntakeWristIO io;
    private WantedState wantedState = WantedState.STOPPED;
    private CurrentState currentState = CurrentState.STOPPED;

    public IntakeWrist(IntakeWristIO io){
        this.io = io;
    }

    enum WantedState{
        STOPPED,
        INTAKING_FUEL,
        STOW,
        AGITATING_FUEL
    }

    enum CurrentState{
        STOPPED,
        INTAKING_FUEL,
        STOWED,
        WRIST_RETRACTING,
        WRIST_EXTENDING
    }

    public void updateInputs(){
        io.updateInputs();
        handleStateTransitions();
        applyStates();
        DogLog.log("Intake/Wrist/WantedState", wantedState);
        DogLog.log("Intake/Wrist/CurrentState", currentState);
    }

    public void handleStateTransitions(){
        switch(wantedState){
            case STOPPED:
                currentState = CurrentState.STOPPED;
                break;
            case INTAKING_FUEL:
                currentState = CurrentState.INTAKING_FUEL;
                break;
            case STOW:
                currentState = CurrentState.STOWED;
                break;
            case AGITATING_FUEL:
                if (currentState == CurrentState.WRIST_RETRACTING) {
                    if (atSetpoint()) {
                        currentState = CurrentState.WRIST_EXTENDING;
                    } else {
                        currentState = CurrentState.WRIST_RETRACTING;
                    }
                if (currentState == CurrentState.WRIST_EXTENDING){
                    if (atSetpoint()){
                        currentState = CurrentState.WRIST_RETRACTING;
                    } else {
                        currentState = CurrentState.WRIST_EXTENDING;
                    }
                    }
                }
                break;
            default:
                currentState = CurrentState.STOPPED;
                break;
        }
    }

    public void applyStates(){
        switch(currentState){
            case STOPPED:
                stop();
                break;
            case INTAKING_FUEL:
                setPosition(0);
                break;
            case STOWED:
                setPosition(0);
                break;
            case WRIST_RETRACTING:
                setPosition(0);
                break;
            case WRIST_EXTENDING:
                setPosition(0);
                break;
            default:
                stop();
                break;

        }
    }

    public void stop(){
        io.stop();
    }

    public void setWantedState(WantedState intakeWristWantedState){
        io.wantedState = intakeWristWantedState;
    }

    public void setPosition(double position){
        io.wantedPosition = position;
    }

    public boolean atSetpoint(){
        //returns a value whether if the wrist is at target position
        return io.atSetpoint;
    }
}
