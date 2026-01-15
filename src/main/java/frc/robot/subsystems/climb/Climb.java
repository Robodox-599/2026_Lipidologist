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
    STOPPED
  }

  public enum ClimbCurrentState{
    STOPPING
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
        default:
            
            break;
    }
  }

  public void applyStates(){
    switch(climbWantedState){
        default:
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

  public void zeroClimbEncoder(){
    io.zeroClimbEncoder();
  }

  public void setClimbWantedState(ClimbWantedState indexerWantedState){
    this.climbWantedState = indexerWantedState;
  }

}