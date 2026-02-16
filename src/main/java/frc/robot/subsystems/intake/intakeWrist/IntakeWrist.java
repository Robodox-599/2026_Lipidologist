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
        INTAKE_FUEL,
        STOW, //pack
        AGITATE_FUEL
    }

    public enum IntakeWristCurrentState{
        STOPPED,
        INTAKING_FUEL,
        STOWING, //packing
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
            case STOP:
                currentState = IntakeWristCurrentState.STOPPED;
                break;
            case INTAKE_FUEL:
                currentState = IntakeWristCurrentState.INTAKING_FUEL;
                break;
            case STOW:
                currentState = IntakeWristCurrentState.STOWING;
                break;
            case AGITATE_FUEL:
                if (currentState == IntakeWristCurrentState.WRIST_RETRACTING){
                    if (atSetpoint()){
                        currentState = IntakeWristCurrentState.WRIST_EXTENDING;
                    } else{
                        currentState = IntakeWristCurrentState.WRIST_RETRACTING;
                    }
                } else if(currentState == IntakeWristCurrentState.WRIST_EXTENDING){
                    if (atSetpoint()){
                        currentState = IntakeWristCurrentState.WRIST_RETRACTING;
                    } else{
                        currentState = IntakeWristCurrentState.WRIST_EXTENDING;
                    } 
                } else {
                    currentState = IntakeWristCurrentState.WRIST_EXTENDING;
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
            case INTAKING_FUEL:
                setPosition(-0.02);
                break;
            case STOWING:
                setPosition(.337);
                break;
            case WRIST_RETRACTING:
                setPosition(0.167);
                break;
            case WRIST_EXTENDING:
                setPosition(0.05);
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
