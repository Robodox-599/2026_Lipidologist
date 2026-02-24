// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  IndexerIO io;
  private IndexerWantedState wantedState = IndexerWantedState.STOP;
  private IndexerCurrentState currentState = IndexerCurrentState.STOPPED;

  public enum IndexerWantedState{
    STOP,
    INDEX,
    REVERSE,
    PULSE,

  }

  public enum IndexerCurrentState{
    STOPPED,
    INDEXING,
    REVERSING,
    PULSING
  }

  public Indexer(IndexerIO io){
    this.io = io;
  }

  public void updateInputs(){
    io.updateInputs();
    handleStateTransitions();
    applyStates();
  }

  private void handleStateTransitions(){
    switch(wantedState){
      case STOP:
        
    }
  }
  
  private void applyStates(){
    switch(currentState){

    }
  }
}