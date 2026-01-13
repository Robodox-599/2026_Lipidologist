// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;

  public enum WantedState{
    RECIEVE,
    TRANSFER_FUEL,
    PULSE_FUEL,
    STOPPED,
  }

  public enum CurrentState{
    RECIEVE,
    TRANSFER_FUEL,
    PULSE_FUEL,
    STOPPED,
  }

  /** Creates a new indexer. */
  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void updateInputs() {
    io.updateInputs();
    
    handleStateTransitions();
    applyStates();

    DogLog.log("Indexer/wantedState", wantedState);
    DogLog.log("Indexer/currentState", currentState);

  }

  public void handleStateTransitions(){
    switch (currentState) {
        case STOPPED:
            currentState = CurrentState.STOPPED;
            break;
        case RECIEVE:
            currentState = CurrentState.RECIEVE;
        case TRANSFER_FUEL:
            currentState = CurrentState.TRANSFER_FUEL;
        case PULSE_FUEL:
            currentState = CurrentState.PULSE_FUEL;
        default:
            currentState = CurrentState.STOPPED;
            break;
    }
  }

  public void applyStates(){
    switch(currentState){
      
      default:
        stop();
        break;
    }
  }

  public void setVelocity(double velocity){
    io.setVelocity(velocity);
  }

  public void setVoltage(double volts){
    io.setVoltage(volts);
  }

  public void stop(){
    io.stop();
  }

  public void setWantedState(WantedState wantedState){
    this.wantedState = wantedState;
  }

}