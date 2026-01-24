// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPING;

  public enum WantedState{
    STOPPED,
    MOVE_LEFT_TO_POSITION,
    MOVE_RIGHT_TO_POSITION,
    MOVE_BOTH_TO_POSITION,
  }

  public enum CurrentState{
    STOPPING,
    MOVING_LEFT_TO_POSITION,
    MOVING_RIGHT_TO_POSITION,
    MOVING_BOTH_TO_POSITION,
  }

  public Climb(ClimbIO io) {
    this.io = io;
  }

  public void updateInputs() {
    io.updateInputs();
    
    handleStateTransitions();
    applyStates();

    DogLog.log("Climb/wantedState", wantedState);
    DogLog.log("Climb/currentState", currentState);

  }

  public void handleStateTransitions(){
    switch (currentState) {
        case STOPPING:
            currentState = CurrentState.STOPPING;
            break;
        case MOVING_LEFT_TO_POSITION:
            currentState = CurrentState.MOVING_LEFT_TO_POSITION;
            break;
        case MOVING_RIGHT_TO_POSITION:
            currentState = CurrentState.MOVING_RIGHT_TO_POSITION;
            break;
        case MOVING_BOTH_TO_POSITION:
            currentState = CurrentState.MOVING_BOTH_TO_POSITION;
            break;
        default:
            currentState = CurrentState.STOPPING;
            break;
    }
  }

  public void applyStates(){
    switch(wantedState){
        case STOPPED:
            stopClimb();
            break;
        case MOVE_LEFT_TO_POSITION:
            setLeftClimbHeight(0);
            break;
        case MOVE_RIGHT_TO_POSITION:
            setRightClimbHeight(0);
            break;
        case MOVE_BOTH_TO_POSITION:
            setLeftClimbHeight(0);
            setRightClimbHeight(0);
            break;
        default:
            stopClimb();
            break;
    }
  }

  public void setLeftClimbVoltage(double volts){
    io.setLeftClimbVoltage(volts);
  }

  public void setRightClimbVoltage(double volts){
    io.setRightClimbVoltage(volts);
  }

  public void setLeftClimbHeight(double height){
    io.setLeftClimbHeight(height);
  }

  public void setRightClimbHeight(double height){
    io.setRightClimbHeight(height);
  }

  public void stopClimb(){
    io.stopClimb();
  }

  public void zeroClimbEncoder(){
    io.zeroClimbPosition();
  }

  public void setWantedState(Climb.WantedState WantedState){
    this.wantedState = WantedState;
  }

}