// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeWrist; // make sure the folder name is all lowercase

import dev.doglog.DogLog;

/** Add your docs here. */
public class IntakeWrist {
    private IntakeWristIO io;
    private WantedState wantedState = WantedState.STOPPED;
    private CurrentState currentState = CurrentState.STOPPED;

    public void intakeWristIO(IntakeWristIO io){
        this.io = io;
    }

    enum WantedState{
        STOPPED,
        INTAKING,
        STOWING,
    }

    enum CurrentState{
        STOPPED,
        INTAKING,
        STOWING
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
            case INTAKING:
                currentState = CurrentState.INTAKING;
                break;
            case STOWING:
                currentState = CurrentState.STOWING;
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
            case INTAKING:
                setPosition(0);
                break;
            case STOWING:
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
}
