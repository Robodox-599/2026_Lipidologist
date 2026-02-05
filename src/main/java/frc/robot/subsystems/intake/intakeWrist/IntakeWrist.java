// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeWrist;

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class IntakeWrist {
    private final IntakeWristIO io;
    private WristWantedState wantedState = WristWantedState.STOP;
    private WristCurrentState currentState = WristCurrentState.STOPPED;

    public IntakeWrist(IntakeWristIO io){
        this.io = io;
    }

    public enum WristWantedState{
        STOP,
        INTAKE_FUEL,
        STOW, //pack
        AGITATE_FUEL
    }

    public enum WristCurrentState{
        STOPPED,
        INTAKING_FUEL,
        STOWED,
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
                currentState = WristCurrentState.STOPPED;
                break;
            case INTAKE_FUEL:
                currentState = WristCurrentState.INTAKING_FUEL;
                break;
            case STOW:
                currentState = WristCurrentState.STOWING;
                break;
            case AGITATE_FUEL:
                if (currentState == WristCurrentState.WRIST_RETRACTING){
                    if (atSetpoint()){
                        currentState = WristCurrentState.WRIST_EXTENDING;
                    } else{
                        currentState = WristCurrentState.WRIST_RETRACTING;
                    }
                } else if(currentState == WristCurrentState.WRIST_EXTENDING){
                    if (atSetpoint()){
                        currentState = WristCurrentState.WRIST_RETRACTING;
                    } else{
                        currentState = WristCurrentState.WRIST_EXTENDING;
                    } 
                } else {
                    currentState = WristCurrentState.WRIST_EXTENDING;
                }
                break;
            default:
                currentState = WristCurrentState.STOPPED;
                break;
        }
    }

    public void applyStates(){
        switch(currentState){
            case STOPPED:
                stop();
                break;
            case INTAKING_FUEL:
                setPosition(0.3);
                break;
            case STOWING:
                setPosition(.2);
                break;
            case WRIST_RETRACTING:
                setPosition(0.2);
                break;
            case STOWED:
                setPosition(0);
            case WRIST_EXTENDING:
                setPosition(0.4);
                break;
            default:
                stop();
                break;

        }
    }

    public void stop(){
        io.stop();
    }

    public void setWantedState(IntakeWrist.WristWantedState wantedState){
        this.wantedState = wantedState;
    }

    public void setPosition(double position){
        io.setPosition(position);
    }

    public boolean atSetpoint(){
        return io.atSetpoint;
    }
}
