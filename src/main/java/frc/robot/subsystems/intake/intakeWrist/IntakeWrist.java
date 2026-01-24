// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeWrist;

/** Add your docs here. */
public class IntakeWrist {
    private IntakeWristIO io;
    private WantedState wantedState = WantedState.STOPPED;
    private CurrentState currentState = CurrentState.STOPPED;

    public IntakeWrist(IntakeWristIO io){
        this.io = io;
    }

    public enum WantedState{
        STOPPED,
        INTAKE_FUEL,
        STOW,
    }

    public enum CurrentState{
        STOPPED,
        INTAKING_FUEL,
        STOWED,
    }

    public void updateInputs(){
        io.updateInputs();
        handleStateTransitions();
        applyStates();
    }

    public void handleStateTransitions(){
        switch(wantedState){
            case STOPPED:
                currentState = CurrentState.STOPPED;
                break;
            case INTAKE_FUEL:
                currentState = CurrentState.INTAKING_FUEL;
                break;
            case STOW:
                currentState = CurrentState.STOWED;
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
