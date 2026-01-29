// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import dev.doglog.DogLog;

public class Climb {
  private final ClimbIO io;
  private WantedState wantedState = WantedState.STOP;
  private CurrentState currentState = CurrentState.STOPPED;

  public enum WantedState{
    STOP,
    EXTEND,
    RETRACT,
  }

  public enum CurrentState{
    STOPPED,
    EXTENDING,
    RETRACTING,
  }

  public Climb(ClimbIO io) {
    this.io = io;
  }

  public void updateInputs() {
    io.updateInputs();
    
    handleStateTransitions();
    applyStates();

    DogLog.log("Climb/WantedState", wantedState);
    DogLog.log("Climb/CurrentState", currentState);

  }

  private void handleStateTransitions(){
    switch (wantedState) {
        case STOP:
            currentState = CurrentState.STOPPED;
            break;
        case EXTEND:
            currentState = CurrentState.EXTENDING;
            break;
        case RETRACT:
            currentState = CurrentState.RETRACTING;
            break;
        default:
            currentState = CurrentState.STOPPED;
            break;
    }
  }

  private void applyStates(){
    switch(currentState){
        case EXTENDING:
            setClimbHeight(50);
            break;
        case RETRACTING:
            setClimbHeight(15);
            break;
        case STOPPED:
            stopClimb();  
            break;
        default:
            stopClimb();
            break;
    }
  }

  public void setClimbVoltage(double volts){
    io.setClimbVoltage(volts);
  }

  public void setClimbHeight(double height){
    io.setClimbHeight(height);
  }

  public void stopClimb(){
    io.stopClimb();
  }

  public void zeroClimbPosition(){
    io.zeroClimbPosition();
  }

  public void setWantedState(Climb.WantedState wantedState){
    this.wantedState = wantedState;
  }

}