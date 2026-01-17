// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private ClimbWantedState climbWantedState = ClimbWantedState.STOPPED;
  private ClimbCurrentState climbCurrentState = ClimbCurrentState.STOPPING;

  public enum ClimbWantedState{
    STOPPED,
    MOVE_LEFT_TO_POSITION,
    MOVE_RIGHT_TO_POSITION,
    MOVE_BOTH_TO_POSITION,
  }

  public enum ClimbCurrentState{
    STOPPING,
    MOVING_LEFT_TO_POSITION,
    MOVING_RIGHT_TO_POSITION,
    MOVING_BOTH_TO_POSITION,
  }

  public Climb(ClimbIO io) {
    this.io = io;
  }

  public void updateClimbInputs() {
    io.updateClimbInputs();
    
    handleStateTransitions();
    applyStates();

    DogLog.log("Climb/wantedState", climbWantedState);
    DogLog.log("Climb/currentState", climbCurrentState);

  }

  public void handleStateTransitions(){
    switch (climbCurrentState) {
        case STOPPING:
            climbCurrentState = ClimbCurrentState.STOPPING;
            break;
        case MOVING_LEFT_TO_POSITION:
            climbCurrentState = ClimbCurrentState.MOVING_LEFT_TO_POSITION;
            break;
        case MOVING_RIGHT_TO_POSITION:
            climbCurrentState = ClimbCurrentState.MOVING_RIGHT_TO_POSITION;
            break;
        case MOVING_BOTH_TO_POSITION:
            climbCurrentState = ClimbCurrentState.MOVING_BOTH_TO_POSITION;
            break;
        default:
            climbCurrentState = ClimbCurrentState.STOPPING;
            break;
    }
  }

  public void applyStates(){
    switch(climbWantedState){
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

  public void setWantedState(ClimbWantedState indexerWantedState){
    this.climbWantedState = indexerWantedState;
  }

}