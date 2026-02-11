// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeWrist;

import dev.doglog.DogLog;

/** Add your docs here. */
public class IntakeWrist {
    private final IntakeWristIO io;
    private IntakeWristWantedState wantedState = IntakeWristWantedState.STOP;
    private IntakeWristCurrentState currentState = IntakeWristCurrentState.STOPPED;

    public IntakeWrist(IntakeWristIO io){
        this.io = io;
    }

    public enum IntakeWristWantedState{
        STOP,
        EXTEND,
        STOW, //pack
        AGITATE_FUEL
    }

    public enum IntakeWristCurrentState{
        STOPPED,
        STOWED,
        STOWING, //packing
        RETRACTING,
        EXTENDING
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
            case STOP:
                currentState = IntakeWristCurrentState.STOPPED;
                break;
            case EXTEND:
                currentState = IntakeWristCurrentState.EXTENDING;
                break;
            case STOW:
                currentState = IntakeWristCurrentState.STOWING;
                break;
            case AGITATE_FUEL:
                if (currentState == IntakeWristCurrentState.RETRACTING){
                    if (atSetpoint()){
                        currentState = IntakeWristCurrentState.EXTENDING;
                    } else{
                        currentState = IntakeWristCurrentState.RETRACTING;
                    }
                } else if(currentState == IntakeWristCurrentState.EXTENDING){
                    if (atSetpoint()){
                        currentState = IntakeWristCurrentState.RETRACTING;
                    } else{
                        currentState = IntakeWristCurrentState.EXTENDING;
                    } 
                } else {
                    currentState = IntakeWristCurrentState.EXTENDING;
                }
                break;
            default:
                currentState = IntakeWristCurrentState.STOPPED;
                break;
        }
    }

    public void applyStates(){
        switch(currentState){
            case STOPPED:
                stop();
                break;
            case EXTENDING:
                setPosition(0.3);
                break;
            case STOWING:
                setPosition(.2);
                break;
            case RETRACTING:
                setPosition(0.2);
                break;
            case STOWED:
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

    public void setWantedState(IntakeWrist.IntakeWristWantedState wantedState){
        this.wantedState = wantedState;
    }

    public void setPosition(double position){
        io.setPosition(position);
    }

    public boolean atSetpoint(){
        return io.atSetpoint;
    }
}
