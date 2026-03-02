// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;
import dev.doglog.DogLog;

public class Climb{
  ClimbIO io;
  public ClimbWantedState wantedState = ClimbWantedState.STOP;
  public ClimbCurrentState currentState = ClimbCurrentState.STOPPED;

  public Climb(ClimbIO io){
    this.io = io;
  }

  public enum ClimbWantedState{
    STOP,
    EXTENDING,
    RETRACTING
  }

  public enum ClimbCurrentState{
    STOPPED,
    EXTENDING,
    RETRACTING
  }

  public void updateInputs(){
    io.updateInputs();
    handleStateTransitions();
    applyStates();

    DogLog.log("Climb/WantedState", wantedState);
    DogLog.log("Climb/CurrentState", currentState);


  }

  private void handleStateTransitions(){
    switch (wantedState){
      case STOP:
        currentState = ClimbCurrentState.STOPPED;
        break;
      case EXTENDING:
        currentState = ClimbCurrentState.EXTENDING;
        if (atSetPoint()){
          currentState = ClimbCurrentState.STOPPED;
          io.targetPosition = 0;
        } else{
          currentState = ClimbCurrentState.EXTENDING;
        }
        break;
      case RETRACTING:
        currentState = ClimbCurrentState.RETRACTING;
        if (atSetPoint()){
          currentState = ClimbCurrentState.STOPPED;
        } else{
          currentState = ClimbCurrentState.RETRACTING;
        }
        break;
    }
  }

  private void applyStates(){
    switch(currentState){
      case STOPPED:
        stop();
        break;
      case EXTENDING:
        setPosition(0);
        break;
      case RETRACTING:
        setPosition(0);
        break;
    }
  }

  private void stop(){
    io.stop();
  }

  private boolean atSetPoint(){
    return io.atSetpoint;
  }

  private void setPosition(double position){
    io.setPosition(position);
  }
}
